#if defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <DNSServer.h>
  #include <ESP8266mDNS.h>
#elif  defined(ESP32)
  #include <WiFi.h>
  #include <DNSServer.h>
  #include "ESPmDNS.h"
#else
  #error "Only supports ESP8266 or ESP32"
#endif

//TODO: must delete util directory of RemoteDebug to get it to compile
//  - need to find a workaround (fork library?)

#include <SocketIOclient.h>
#include <WiFiManager.h>
#include <ArduinoOTA.h>
#include <SPI.h>
#include <time.h>
#include "secrets.h"

#define USE_LIB_WEBSOCKET true
#define WEBSOCKET_DISABLED true
#include "RemoteDebug.h"

#define HOST_NAME "smu5560"

/****************************************
 * Time and Time Zone
 ***************************************/
#define NTP_SERVER "pool.ntp.org"
#define TZ "EST5EDT,M3.2.0,M11.1.0" // https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv

time_t now;
tm timeinfo;

/****************************************
 * Wifi manager
 ***************************************/
WiFiManager wm;

/****************************************
 * OTA Service
 ***************************************/
const char *OTAName     = SECRET_OTA_NAME;
const char *OTAPassword = SECRET_OTA_PASS;

/****************************************
 *  SPI Config
 ***************************************/
#define SPI_ADC
//#define SPI_INAMP
//#define SPI_DPS



/****************************************
 *  ADC Config (AD7717)
 ***************************************/
const int PIN_ADC_SCLK = 14; // GPIO14 (D5) - SCLK (Drive high between transactions)
                             // GPIO13 (D7) - MOSI
const int PIN_ADC_CS   =  4; // GPIO4  (D2) - CSb  (Not controlled by SPI library)
const int PIN_ADC_MISO = 12; // GPIO12 (D6) - MISO (Used as DATA READY signal)
volatile bool adc_data = false;

/****************************************
 *  Inamp Config (ADA4254)
 ***************************************/
const int PIN_AMP_SCLK = 14; // GPIO14 (D5) - SCLK (Drive high between transactions)
                             // GPIO13 (D7) - MOSI
const int PIN_AMP_CS   =  4; // GPIO4  (D2) - CSb  (Not controlled by SPI library)
const int PIN_AMP_MISO = 12; // GPIO12 (D6) - MISO (Used as DATA READY signal)

/****************************************
 *  DPS Config (AD5560) - SPI Mode 1
 ***************************************/
const int PIN_DPS_SCLK = 14; // GPIO14 (D5) - SCLK (Drive high between transactions)
                             // GPIO13 (D7) - MOSI
const int PIN_DPS_CS   =  4; // GPIO4  (D2) - CSb  (Not controlled by SPI library)
const int PIN_DPS_MISO = 12; // GPIO12 (D6) - MISO (Used as DATA READY signal)


/****************************************
 *  Remote debug
 ***************************************/
// Disable remote debug for production build
//#define DEBUG_DISABLED true

#ifndef DEBUG_DISABLED
  RemoteDebug Debug;
#endif

/**********************************************************
 *
 * MDNS Functions
 *
 *********************************************************/

// Setup multicast DNS name
void mdns_init() {
  String hostNameWifi = HOST_NAME;
  hostNameWifi.concat(".local");
  WiFi.hostname(hostNameWifi);
  if (MDNS.begin(HOST_NAME)) {
    Serial.print("* MDNS responder started. Hostname -> ");
    Serial.println(HOST_NAME);
  }
}

/**********************************************************
 *
 * OTA Functions
 *
 *********************************************************/
void ota_init() {
  ArduinoOTA.setHostname(OTAName);
  ArduinoOTA.setPassword(OTAPassword);

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\r\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("OTA ready\r\n");
}

/**********************************************************
 *
 * ADC SPI Functions
 *
 *********************************************************/
#ifdef SPI_ADC

void IRAM_ATTR adc_data_ready() {
  adc_data = true; // Set flag when data ready
}

void adc_start_interrupt() {
    // Drive SCLK high between transactions
    pinMode(PIN_ADC_SCLK, OUTPUT);
    digitalWrite(PIN_ADC_SCLK, HIGH);

    // Reselect ADC so interrupt can occur
    pinMode(PIN_ADC_CS, OUTPUT);
    digitalWrite(PIN_ADC_CS, LOW);

    // Enable interrupt after SPI transaction
    pinMode(PIN_ADC_MISO, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_ADC_MISO), adc_data_ready, FALLING);
}

void adc_stop_interrupt() {
  // Deselect ADC before configuring SPI
  digitalWrite(PIN_ADC_CS, HIGH);

  // Disable interrupt before starting SPI transaction
  detachInterrupt(digitalPinToInterrupt(PIN_ADC_MISO));
}

uint64_t adc_transaction(uint8_t cmd, uint64_t data, uint32_t num_bits) {
  uint32_t num_bytes;
  uint64_t ret = 0;

  // Check that len is multiple of 8
  if (num_bits % 8 == 0){
    num_bytes = num_bits / 8;

    adc_stop_interrupt();

    // Initialize SPI (Default pins: SCK=GPIO14, MOSI=GPIO13, MISO=GPIO12)
    SPI.begin();

    // Start SPI transaction (1MHz)
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));

    // Select the ADC
    digitalWrite(PIN_ADC_CS, LOW);

    // Send ADC SPI command
    SPI.transfer(cmd);

    // Read/write ADC data
    for (uint32_t i = 0; i < num_bytes; i++){
      uint8_t write_byte, read_byte;

      write_byte = (data >> (8*(num_bytes-1-i))) & 0xFF;
      read_byte = SPI.transfer(write_byte);

      ret = ret | (read_byte << (8*(num_bytes-1-i)));
    }

    // Deselect the ADC while ending SPI control
    digitalWrite(PIN_ADC_CS, HIGH);

    // End SPI transaction
    SPI.endTransaction();

    // Release SPI bus so we can add interrupt on MISO pin
    SPI.end();

    // Reconfig pins for MISO interrupt
    adc_start_interrupt();

    // Print the result (for debugging purposes)
    debugD("ADC transaction: cmd - 0x%X, read - 0x%X", cmd, ret);
  } else {
    // Function call error
    debugE("ADC transaction len must be multiple of 8.\n"
      "\t adc_transaction(0x%x, 0x%x, %d)", cmd, data, num_bits);
  }

  return ret;
}

void adc_init() {
  // Setup pins for ADC MISO interrupt
  adc_start_interrupt();

  // TODO configure ADC
  // Set ch0 to 5 SPS
  //adc_transaction(0x28, 0x0514, 16);

  // Set ch0 to 1kSPS
  adc_transaction(0x28, 0x050A, 16);

  // Set ch0 to 10kSPS
  //adc_transaction(0x28, 0x0507, 16);

  // Read ID register (16 bits) - should read 0x4FDX
  adc_transaction(0x47, 0x0, 16);
}

void adc_process(){
  if (adc_data){
    adc_data = false;

    // Read data register (24 bits)
    adc_transaction(0x44, 0x0, 24);
  }
}
#endif

/**********************************************************
 *
 * INAMP SPI Functions
 *
 *********************************************************/
#ifdef SPI_INAMP

uint64_t inamp_transaction(uint8_t rw, uint8_t cmd, uint8_t data, uint32_t num_bits) {
  uint64_t ret;
  return ret;
}

void inamp_init() {
}

void inamp_process(){
}
#endif

/**********************************************************
 *
 * DPS SPI Functions
 *
 *********************************************************/
#ifdef SPI_DPS

uint64_t dps_transaction(uint8_t rw, uint8_t cmd, uint8_t data, uint32_t num_bits) {
  uint32_t num_bytes;
  uint64_t ret = 0;

  // Check that len is multiple of 8
  if (num_bits % 8 == 0){
    num_bytes = num_bits / 8;

    adc_stop_interrupt();

    // Initialize SPI (Default pins: SCK=GPIO14, MOSI=GPIO13, MISO=GPIO12)
    SPI.begin();

    // Start SPI transaction (1MHz)
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));

    // Select the ADC
    digitalWrite(PIN_ADC_CS, LOW);

    // Send ADC SPI command
    SPI.transfer(cmd);

    // Read/write ADC data
    for (uint32_t i = 0; i < num_bytes; i++){
      uint8_t write_byte, read_byte;

      write_byte = (data >> (8*(num_bytes-1-i))) & 0xFF;
      read_byte = SPI.transfer(write_byte);

      ret = ret | (read_byte << (8*(num_bytes-1-i)));
    }

    // Deselect the ADC while ending SPI control
    digitalWrite(PIN_ADC_CS, HIGH);

    // End SPI transaction
    SPI.endTransaction();

    // Release SPI bus so we can add interrupt on MISO pin
    SPI.end();

    // Reconfig pins for MISO interrupt
    adc_start_interrupt();

    // Print the result (for debugging purposes)
    debugD("ADC transaction: cmd - 0x%X, read - 0x%X", cmd, ret);
  } else {
    // Function call error
    debugE("ADC transaction len must be multiple of 8.\n"
      "\t adc_transaction(0x%x, 0x%x, %d)", cmd, data, num_bits);
  }

  return ret;
  uint64_t ret;
  return ret;
}

void dps_init() {
}

void dps_process(){
}
#endif

/**********************************************************
 *
 * RemoteDebug Functions
 *
 *********************************************************/
void debug_init() {
  Debug.begin(HOST_NAME);         // Initialize the WiFi server
  Debug.setResetCmdEnabled(true); // Enable the reset command
  Debug.showProfiler(true);       // Enable time profiling
  Debug.showColors(true);         // Enable olors
  MDNS.addService("telnet", "tcp", 23);
}

void debug_process() {
  Debug.handle();       // remote debug
  String last_cmd = Debug.getLastCommand();
  Debug.clearLastCommand();
}

/**********************************************************
 *
 * Main Functions
 *
 *********************************************************/

uint32_t millis_last;
const int PIN_TEST   =  5; // GPIO5  (D1)

void setup() {
  // Start wifi in station mode
  WiFi.mode(WIFI_STA);

  // Start serial port
  Serial.begin(115200);

  // Start MDNS
  mdns_init();

  // Start remote debug
  debug_init();

  // Configure timezone and ntp server
  configTime(TZ, NTP_SERVER);

  // Setup wifimanager
  // TODO figure out how to make this non-blocking
  wm.autoConnect("SetupSMU5560");

  // Setup OTA service
  ota_init();

  #ifdef SPI_ADC
  // Setup ADC SPI
  adc_init();
  #endif

  //TODO: test/debug stuff
  millis_last = millis();
  pinMode(PIN_TEST, OUTPUT);
  digitalWrite(PIN_TEST, HIGH);
}

void loop() {
  // Handle library processing functions
  debug_process();
  ArduinoOTA.handle();  // OTA update

  #ifdef SPI_ADC
  adc_process();
  #endif

  yield();              // ESP processing time


  if (millis() - millis_last > 2000) {
    #ifdef SPI_ADC
    // Read ID register (16 bits) - should read 0x4FDX
    //adc_transaction(0x47, 0x0, 16);
    #endif

    digitalWrite(PIN_TEST, !digitalRead(PIN_TEST));
    millis_last = millis();
  }

}
