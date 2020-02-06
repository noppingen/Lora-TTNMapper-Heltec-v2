/*
-----------------------------------------------------------------------------------------------------------
 TTNMapper node with GPS running on an Heltec "WiFi Lora32 v2": https://heltec.org/project/wifi-lora-32/

 Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman: https://github.com/matthijskooijman/arduino-lmic/blob/master/examples/ttn-abp/ttn-abp.ino
 Copyright (c) 2018 sbiermann: https://github.com/sbiermann/Lora-TTNMapper-ESP32
 Copyright (c) 2019 sistemasorp: https://github.com/sistemasorp/Heltec-Wifi-Lora-32-TTN-Mapper
 Copyright (c) 2020 Stefan Onderka: https://www.onderka.com/computer-und-netzwerk/ttnmapper-node-mit-heltec-sx1276-lora-esp32-v2
 Case: https://www.thingiverse.com/thing:4145143

 This code expects a serial connected GPS module like an U-Blox NEO-6m, you may change baud rate and pins. 
 Default is GPIO 2 Rx and GPIO 17 Tx (Closest to "end" of board), speed 9600 Baud. Use 3V3 or Vext for GPS power
 
 Set your gateway GPS coordinates to show distance and direction when mapping.

 The activation method of the TTN device has to be ABP. 

 Libraries needed: U8x8 (From U8g2), TinyGPS++, IBM LMIC, SPI, Wifi
-----------------------------------------------------------------------------------------------------------
*/

/* Hardware serial */
#include <HardwareSerial.h>
/* GPS */
#include <TinyGPS++.h>
/* LoraMAC-in-C */
#include <lmic.h>
/* Hardware abstraction layer */
#include <hal/hal.h>
/* SPI */
#include <SPI.h>
/* OLED */
#include <U8x8lib.h>
/* Wireless */
#include "WiFi.h"

/* Integrated LED (white) */
#define BUILTIN_LED 25
/* GPS Rx pin */
#define GPS_RX 17
/* GPS Tx pin */  
#define GPS_TX 2
/* GPS baud rate */
#define GPS_BAUD 9600

/* GPS coordinates of mapped gateway for calculating the distance */
const double HOME_LAT = 49.000000;
const double HOME_LNG = 11.000000;

/* Initial sending interval in seconds */
const unsigned TX_INTERVAL = 15;

/* Upper TinyGPS++ HDOP value limit to send Pakets with */
int HDOP_MAX = 300; // Set to 200-500 (HDOP 2.00 - 5.00)

/* Time to wait in seconds if HDOP is above limit */
const unsigned TX_WAIT_INTERVAL = 5;

/* Define serial for GPS */
HardwareSerial GPSSerial(1);

/* Init GPS */
TinyGPSPlus gps;

/* Define OLED (RST, SCL, SDA - See pinout) */
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(15, 4, 16);

/* LoRaWAN network session key  */
static const PROGMEM u1_t NWKSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; 
/* LoRaWAN application session key */ 
static const u1_t PROGMEM APPSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; 
/* LoRaWAN device address */
static const u4_t DEVADDR = 0xffffffff; 

/* These callbacks are only used in over-the-air activation, so they are left empty here. We cannot  */
/* leave them out completely unless DISABLE_JOIN is set in config.h, otherwise the linker will complain */
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

/* Pin mapping */
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {26, 34, 35}, /* Board V2 */
};

/* Buffer with data to send to TTNMapper */
uint8_t txBuffer[9];

/* Buffer for sending to serial line*/
char sbuf[16];

void build_packet() {
  /* For sprintf to OLED display */
  char s[16]; 
  String toLog;
  
  uint32_t LatitudeBinary, LongitudeBinary;
  uint16_t altitudeGps;
  uint8_t hdopGps;
  
  while (GPSSerial.available())
    gps.encode(GPSSerial.read());

  /* Get GPS data */
  double latitude = gps.location.lat();
  double longitude = gps.location.lng();
  double altitude = gps.altitude.meters();
  uint32_t hdop = gps.hdop.value();

  /* For OLED display */
  float hdopprint = (float)hdop / 100.0;
  uint32_t sats = gps.satellites.value();
  uint32_t characters = gps.charsProcessed();
  uint32_t fixes = gps.sentencesWithFix();
  double failedchecksums = gps.failedChecksum();
  double passedchecksums = gps.passedChecksum();
  float speed = gps.speed.kmph();

  /* Serial output */
  Serial.println();

  /* Distance to GW in Meters */
  double fromhome = gps.distanceBetween(gps.location.lat(), gps.location.lng(), HOME_LAT, HOME_LNG);

  /* Course to GW */
  double direction_home = gps.courseTo(gps.location.lat(), gps.location.lng(), HOME_LAT, HOME_LNG);

  Serial.print("Latitude:             ");
  sprintf(s, "%f", latitude);
  Serial.println(s);
  //Serial.println(latitude);
  Serial.print("Longitude:            ");
  sprintf (s, "%f", longitude);
  Serial.println(s);
  //Serial.println(longitude);
  Serial.print("Altitude (m):         ");
  Serial.println(altitude);
  Serial.print("Speed (km/h):         ");
  Serial.println(speed);
  Serial.print("HDOP:                 ");
  Serial.println(hdopprint);
  Serial.print("Satellite count:      ");
  Serial.println(sats);
  if ( fromhome < 1000) {
    // Less than 1000 m
    Serial.print("Distance to GW (m):   ");
    Serial.println(fromhome);
  } else {
    // More than 1 km
    Serial.print("Distance to GW (km):  ");
    Serial.println(fromhome/1000);
  }
  Serial.print("Direction to GW:      ");
  Serial.print((String)gps.cardinal(direction_home) + " (" + (String)direction_home + "°)");

  /*
  Serial.print("Characters processed: ");
  Serial.println(characters);
  Serial.print("Sentences with fix:   ");
  Serial.println(fixes);
  Serial.print("Checksum passed:      ");
  Serial.println(passedchecksums);
  Serial.print("Checksum failed:      ");
  Serial.println(failedchecksums);
  */

  LatitudeBinary = ((latitude + 90) / 180.0) * 16777215;
  LongitudeBinary = ((longitude + 180) / 360.0) * 16777215;

  /* OLED output */
  // Latitude
  u8x8.clearLine(1);
  u8x8.setCursor(0, 1);
  u8x8.print("Lati ");
  u8x8.setCursor(5, 1);
  sprintf(s, "%f", latitude);
  u8x8.print(s);

  // Longitude
  u8x8.clearLine(2);
  u8x8.setCursor(0, 2);
  u8x8.print("Long ");
  u8x8.setCursor(5, 2);
  sprintf(s, "%f", longitude);
  u8x8.print(s);

  // Altitude
  u8x8.clearLine(3);
  u8x8.setCursor(0, 3);
  u8x8.print("Alti ");
  u8x8.setCursor(5, 3);
  sprintf(s, "%f", altitude);
  u8x8.print(s);

  // Number of fixes
  /*
  u8x8.clearLine(4);
  u8x8.setCursor(0, 4);
  u8x8.print("Fixs ");
  u8x8.setCursor(5, 4);
  u8x8.print(fixes);
  */

  // Distance from home GW
  u8x8.clearLine(4);
  u8x8.setCursor(0, 4);
  u8x8.print("Dist ");
  u8x8.setCursor(5, 4);
  if ( fromhome < 1000) {
    // Less than 1000 m
    sprintf(s, "%.0f m", fromhome);
    // More than 1 km
  } else {
    sprintf(s, "%.3f km", fromhome/1000);
  }
  u8x8.print(s);

  // HDOP
  u8x8.clearLine(5);
  u8x8.setCursor(0, 5);
  u8x8.print("HDOP ");
  u8x8.setCursor(5, 5);
  u8x8.print(hdopprint);

  // Number of satellites
  u8x8.clearLine(6);
  u8x8.setCursor(0, 6);
  u8x8.print("Sats ");
  u8x8.setCursor(5, 6);
  u8x8.print(sats);

  txBuffer[0] = ( LatitudeBinary >> 16 ) & 0xFF;
  txBuffer[1] = ( LatitudeBinary >> 8 ) & 0xFF;
  txBuffer[2] = LatitudeBinary & 0xFF;

  txBuffer[3] = ( LongitudeBinary >> 16 ) & 0xFF;
  txBuffer[4] = ( LongitudeBinary >> 8 ) & 0xFF;
  txBuffer[5] = LongitudeBinary & 0xFF;

  altitudeGps = altitude;
  txBuffer[6] = ( altitudeGps >> 8 ) & 0xFF;
  txBuffer[7] = altitudeGps & 0xFF;

  hdopGps = hdop/10;
  txBuffer[8] = hdopGps & 0xFF;

  toLog = "";
  for(size_t i = 0; i<sizeof(txBuffer); i++) {
    char buffer[3];
    sprintf(buffer, "%02x", txBuffer[i]);
    toLog = toLog + String(buffer);
  }

  Serial.println();

  /* Print Tx buffer on serial console */
  // Serial.println(toLog);
}

void onEvent (ev_t ev) {
  /* Clear OLED line #8 */
  u8x8.clearLine(7);
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("> EV_SCAN_TIMEOUT"));
      u8x8.drawString(0, 7, "EV_SCAN_TIMEOUT ");
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("> EV_BEACON_FOUND"));
      u8x8.drawString(0, 7, "EV_BEACON_FOUND ");
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("> EV_BEACON_MISSED"));
      u8x8.drawString(0, 7, "EV_BEACON_MISSED");
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("> EV_BEACON_TRACKED"));
      u8x8.drawString(0, 7, "EV_BEACON_TRACKD");
      break;
    case EV_JOINING:
      Serial.println(F("> EV_JOINING"));
      u8x8.drawString(0, 7, "EV_JOINING      ");
      break;
    case EV_JOINED:
      Serial.println(F("> EV_JOINED"));
      u8x8.drawString(0, 7, "EV_JOINED       ");
      /* Disable link check validation (automatically enabled during join, but not supported by TTN at this time). */
      LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      Serial.println(F("> EV_RFU1"));
      u8x8.drawString(0, 7, "EV_RFUI         ");
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("> EV_JOIN_FAILED"));
      u8x8.drawString(0, 7, "EV_JOIN_FAILED  ");
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("> EV_REJOIN_FAILED"));
      u8x8.drawString(0, 7, "EV_REJOIN_FAILED");
      //break;
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("> EV_TXCOMPLETE (including wait for RX window)"));
      u8x8.drawString(0, 7, "EV_TXCOMPLETE   ");
      digitalWrite(BUILTIN_LED, LOW);
      if (LMIC.txrxFlags & TXRX_ACK) {
        /* Received ACK */
        Serial.println(F("Received ack"));
        u8x8.drawString(0, 7, "ACK RECEIVED    ");
      }
      if (LMIC.dataLen) {
        /* Received data */
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes payload"));

        u8x8.clearLine(6);
        u8x8.drawString(0, 6, "RX ");
        u8x8.setCursor(3, 6);
        u8x8.printf("%i bytes", LMIC.dataLen);

        u8x8.setCursor(0, 7);
        u8x8.printf("RSSI %d SNR %.1d", LMIC.rssi, LMIC.snr);
      }
      /* Schedule next transmission in TX_INTERVAL seconds */
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("> EV_LOST_TSYNC"));
      u8x8.drawString(0, 7, "EV_LOST_TSYNC   ");
      break;
    case EV_RESET:
      Serial.println(F("> EV_RESET"));
      u8x8.drawString(0, 7, "EV_RESET        ");
      break;
    case EV_RXCOMPLETE:
      Serial.println(F("> EV_RXCOMPLETE"));
      u8x8.drawString(0, 7, "EV_RXCOMPLETE   ");
      break;
    case EV_LINK_DEAD:
      Serial.println(F("> EV_LINK_DEAD"));
      u8x8.drawString(0, 7, "EV_LINK_DEAD    ");
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("> EV_LINK_ALIVE"));
      u8x8.drawString(0, 7, "EV_LINK_ALIVE   ");
      break;
    default:
      Serial.println(F("Unknown event"));
      u8x8.setCursor(0, 7);
      u8x8.printf("UNKNOWN EVT %d", ev);
      break;
  }
}

void do_send(osjob_t* j) {
  /* Check for running Tx/Rx job */
  if (LMIC.opmode & OP_TXRXPEND) {
    /* Tx pending */
    Serial.println(F("> OP_TXRXPEND, not sending"));
    u8x8.drawString(0, 7, "OP_TXRXPEND, not sent");
  } else {
    /* No pending Tx, Go! */
    build_packet();
    if ( ((int)gps.hdop.value() < HDOP_MAX) && ((int)gps.hdop.value() != 0) ) {
      /* GPS OK, prepare upstream data transmission at the next possible time */
      //build_packet();
      LMIC_setTxData2(1, txBuffer, sizeof(txBuffer), 0);
      Serial.println();
      Serial.println(F("> PACKET QUEUED"));
      u8x8.drawString(0, 7, "PACKET_QUEUED  ");
      digitalWrite(BUILTIN_LED, HIGH);
    } else { 
      /* GPS not ready */
      //Serial.println();
      Serial.println(F("> GPS NOT READY, not sending"));
      //Serial.println(gps.hdop.value());
      u8x8.drawString(0, 7, "NO_GPS_WAIT    ");
      /* Schedule next transmission in TX_WAIT_INTERVAL seconds */
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_WAIT_INTERVAL), do_send);
    }
  } 
  /* Next TX is scheduled after TX_COMPLETE event. */
}

void setup() {
  /* Start serial */
  Serial.begin(115200);

  /* Turn off WiFi and Bluetooth */
  WiFi.mode(WIFI_OFF);
  btStop();

  /* Turn on Vext pin fpr GPS power */
  Serial.println("Turning on Vext pins for GPS ...");
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);

  Serial.println("Waiting 2 seconds for GPS to power on ...");
  delay(2000);

  /* Initialize GPS */
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);

  /* Initialize OLED */
  u8x8.begin();
  u8x8.setFont(u8x8_font_chroma48medium8_r);

  /* Initialize SPI */
  SPI.begin(5, 19, 27);

  /* Initialize LMIC */
  os_init();

  /* Reset the MAC state. Session and pending data transfers will be discarded. */
  LMIC_reset();

  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);

  /* Define all usable channels and data rates (SF) on EU_868 */
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band

  /* For testing: Define a fixed single channel and data rate (SF) to use */
  // int channel = 0;

  /* For Testing: Disable all channels, except for the one defined above. */
  /*
  for(int i=0; i<9; i++) { // "i<9" For EU; for US use "i<71"
    if(i != channel) {
      LMIC_disableChannel(i);
    }
  }
  */
  
  /* Disable link check validation */
  LMIC_setLinkCheckMode(0);

  /* TTN uses SF9 for its RX2 window. */
  LMIC.dn2Dr = DR_SF9;

  /* Set data rate SF7 for mapping and transmit power for uplink (note: TXpow seems to be ignored by the library) */
  LMIC_setDrTxpow(DR_SF7,14); 
  
  /* Start sending job */
  do_send(&sendjob);
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);
}

void loop() {
  /* The loop */
  os_runloop_once();
}
