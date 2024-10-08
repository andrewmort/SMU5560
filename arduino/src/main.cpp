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
#define SECRET_OTA_NAME "SMU5560_DEV"
#define SECRET_OTA_PASS "SMU5560_DEV"
const char *OTAName     = SECRET_OTA_NAME;
const char *OTAPassword = SECRET_OTA_PASS;

/****************************************
 *  SPI Config
 ***************************************/
#define SPI_ADC
#define SPI_INAMP
#define SPI_DPS

#if defined(ESP8266)
  // Default SPI Pins
  #define PIN_SPI_MISO 12 // GPIO12 (D6)
  #define PIN_SPI_MOSI 13 // GPIO13 (D7)
  #define PIN_SPI_SCLK 14 // GPIO14 (D5)
  #define PIN_ADC_CSB   4 // GPIO4 (D2)
#elif  defined(ESP32)
  // Default VSPI Pins
  #define PIN_SPI_MISO 19
  #define PIN_SPI_MOSI 23
  #define PIN_SPI_SCLK 18
  #define PIN_ADC_CSB   5
  #define PIN_INAMP_CSB  15
  #define PIN_DPS_SYNCB  32
  // Default HSPI Pins
  //#define PIN_SPI_MISO 12
  //#define PIN_SPI_MOSI 13
  //#define PIN_SPI_SCLK 14
  //#define PIN_SPI_CS   15
#endif


/****************************************
 *  ADC Config (AD7717)
 ***************************************/
const int PIN_ADC_SCLK = PIN_SPI_SCLK;
const int PIN_ADC_CS   = PIN_ADC_CSB; // Not controlled by SPI library
const int PIN_ADC_MISO = PIN_SPI_MISO;
volatile bool adc_data = false;

/****************************************
 *  Inamp Config (ADA4254)
 ***************************************/
const int PIN_INAMP_SCLK = PIN_SPI_SCLK;
const int PIN_INAMP_CS   = PIN_INAMP_CSB; // Not controlled by SPI library
const int PIN_INAMP_MISO = PIN_SPI_MISO;
const int PIN_INAMP_ERR  = 33; // Connect to GPIO3

/****************************************
 *  DPS Config (AD5560) - SPI Mode 1
 ***************************************/
const int PIN_DPS_SCLK  = PIN_SPI_SCLK;
const int PIN_DPS_CS    = PIN_DPS_SYNCB; // Not controlled by SPI library
const int PIN_DPS_MISO  = PIN_SPI_MISO;
const int PIN_DPS_RESET =  4; // GPIO4 has weak pull down at reset
const int PIN_DPS_BUSY  = 25; // GPIO25


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
void dns_init() {
  String hostNameWifi = HOST_NAME;
  hostNameWifi.concat(".local");

  #if defined(ESP8266)
    WiFi.hostname(hostNameWifi);
  #elif  defined(ESP32)
    WiFi.setHostname(hostNameWifi.c_str());
  #endif

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
 * General SPI Functions
 *
 *********************************************************/
void spi_init(){
  // Set DPS csb high
  pinMode(PIN_DPS_CS, OUTPUT);
  digitalWrite(PIN_DPS_CS, HIGH);

  // Set ADC csb high
  pinMode(PIN_ADC_CS, OUTPUT);
  digitalWrite(PIN_ADC_CS, HIGH);

  // Set AMP csb high
  pinMode(PIN_INAMP_CS, OUTPUT);
  digitalWrite(PIN_INAMP_CS, HIGH);
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
  //TODO
  //return;

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
  //TODO
  //return;

  // Deselect ADC before configuring SPI
  digitalWrite(PIN_ADC_CS, HIGH);

  // Disable interrupt before starting SPI transaction
  detachInterrupt(digitalPinToInterrupt(PIN_ADC_MISO));
}

// rw = 0 for write, 1 for read
uint64_t adc_transaction(uint8_t rw, uint8_t cmd, uint64_t data, uint32_t num_bits) {
  uint32_t num_bytes;
  uint64_t ret = 0;

  // Check that len is multiple of 8
  if (num_bits % 8 == 0){
    num_bytes = num_bits / 8;

    adc_stop_interrupt();

    // Initialize SPI
    #if defined(ESP8266)
    SPI.begin();
    #elif  defined(ESP32)
    SPI.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI);
    //debugD("ADC begin SPI");
    #endif

    // Start SPI transaction (1MHz)
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));

    // Select the ADC
    digitalWrite(PIN_ADC_CS, LOW);

    // Send ADC SPI command
    SPI.transfer(cmd | ((rw & 0x1) << 6));

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
    //debugD("ret = 0x%X", ret);

    // Reconfig pins for MISO interrupt
    adc_start_interrupt();

    // Print the result (for debugging purposes)
    //debugD("ADC transaction: read = 0x%X", ret);
    //debugD("ADC transaction: data = 0x%X, read = 0x%X", data, ret);
    //debugD("ADC transaction: cmd = 0x%X, data = 0x%X, read = 0x%X", cmd, data, ret);
    //debugD("ADC transaction: rw = %d, cmd = 0x%X, data = 0x%X, read = 0x%X", rw, cmd, data, ret);

    //debugD("ret = 0x%X", ret);
  } else {
    // Function call error
    debugE("ADC transaction len must be multiple of 8.\n"
      "\t adc_transaction(%d, 0x%x, 0x%x, %d)", rw, cmd, data, num_bits);
  }

  return ret;
}

void adc_write(uint8_t addr, uint64_t data, uint32_t num_bits) {
  adc_transaction(0, addr, data, num_bits);
}

uint64_t adc_read(uint8_t addr, uint32_t num_bits) {
  return adc_transaction(1, addr, 0x00, num_bits);
}

void adc_init() {
  // Setup pins for ADC MISO interrupt
  adc_start_interrupt();

  // TODO configure ADC
  // Set ch0 to 5 SPS
  adc_write(0x28, 0x0514, 16);

  // Set ch0 to 1kSPS
  //adc_write(0x28, 0x050A, 16);

  // Set ch0 to 10kSPS
  //adc_write(0x28, 0x0507, 16);

  // Read ID register (16 bits) - should read 0x4FDX
  //adc_read(0x07, 16);

  // Append status to data read
  adc_write(0x02, 0x0040, 16);
}

void adc_process(){
  uint64_t read = 0;

  if (adc_data){
    adc_data = false;

    // Read data register (24 bits) + status (8bits)
    read = adc_read(0x04, 32);
    debugD("Read ADC: 0x%X", read);
    //read = adc_read(0x00, 8);
    //debugD("ADC Status: 0x%X", read);
  }
}
#endif

/**********************************************************
 *
 * INAMP SPI Functions
 *
 *********************************************************/
#ifdef SPI_INAMP

// rw = 0 for write, 1 for read
uint8_t inamp_transaction(uint8_t rw, uint8_t cmd, uint8_t data) {
  uint8_t ret;

  // Initialize SPI
  #if defined(ESP8266)
  SPI.begin();
  #elif  defined(ESP32)
  SPI.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI);
  #endif

  // Start SPI transaction (1MHz)
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  // Select the AMP
  digitalWrite(PIN_INAMP_CS, LOW);

  // Send ADC SPI command
  SPI.transfer(cmd | (rw & 0x1) << 7);

  // Write/read
  ret = SPI.transfer(data);

  // Deselect the AMP while ending SPI control
  digitalWrite(PIN_INAMP_CS, HIGH);

  // End SPI transaction
  SPI.endTransaction();

  // Release SPI bus
  SPI.end();

  // Print the result (for debugging purposes)
  //debugD("INAMP transaction: rw = %d, cmd = 0x%X, data = 0x%X, read = 0x%X", rw, cmd, data, ret);

  return ret;
}

void inamp_write(uint8_t addr, uint8_t data) {
  inamp_transaction(0, addr, data);
}

uint8_t inamp_read(uint8_t addr) {
  return inamp_transaction(1, addr, 0x00);
}

void IRAM_ATTR inamp_error() {
  uint8_t err;
  err = inamp_read(0x03);
  debugD("inamp_error: DIGITAL_ERR (0x03) - 0x%X", err);
  err = inamp_read(0x04);
  debugD("inamp_error: ANALOG_ERR  (0x04) - 0x%X", err);
}

void inamp_init() {
  // Set AMP csb high
  pinMode(PIN_INAMP_CS, OUTPUT);
  digitalWrite(PIN_INAMP_CS, HIGH);

  // GAIN_MUX (0x00): Gain = 1x
  inamp_write(0x00, (0x4 << 3));

  // GPIO_DIR (0x08): Set GPIO3 as output for error detection
  inamp_write(0x08, (1 << 3));

  // SF_CFG (0x0C): Enable Fault Interrupt Output on GPIO3
  inamp_write(0x0C, (1 << 3));

  //TODO scheduled calibration? (reg 0x0E)

  // Read ID register - should read 0x30
  inamp_read(0x2F);

  // Enable error interrupt
  pinMode(PIN_INAMP_ERR, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_INAMP_ERR), inamp_error, RISING);
}

#endif

/**********************************************************
 *
 * DPS SPI Functions
 *
 *********************************************************/
#ifdef SPI_DPS

uint16_t dps_transaction(uint8_t rw, uint8_t addr, uint16_t data) {
  uint16_t ret;

  // Initialize SPI
  #if defined(ESP8266)
  SPI.begin();
  #elif  defined(ESP32)
  SPI.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI);
  #endif

  // Start SPI transaction (1MHz)
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));

  // Select the DPS
  digitalWrite(PIN_DPS_CS, LOW);

  // Send SPI command
  SPI.transfer(addr | (rw & 0x1) << 7);

  // Write data
  for (uint32_t i = 0; i <= 1; i++){
    uint8_t write_byte;

    write_byte = (data >> (8*(1-i))) & 0xFF;
    SPI.transfer(write_byte);
  }

  // SYNC toggle between write/read
  digitalWrite(PIN_DPS_CS, HIGH);
  delayMicroseconds(1);
  digitalWrite(PIN_DPS_CS, LOW);

  // Read data
  if (rw) {
    for (uint32_t i = 0; i <= 2; i++){
      uint8_t read_byte;

      read_byte = SPI.transfer(0x00);
      ret = ret | (read_byte << (8*(2-i)));
    }
  }

  //TODO hold CSb low until busyb low? add timeout.
  while(digitalRead(PIN_DPS_BUSY) == LOW) {
    debugD("DPS transaction: Delaying for busyb");
    delay(1);
  }

  // Deselect the DPS while ending SPI control
  digitalWrite(PIN_DPS_CS, HIGH);

  // End SPI transaction
  SPI.endTransaction();

  // Release SPI bus
  SPI.end();

  // Print the result (for debugging purposes)
  //debugD("DPS transaction: rw = %d, addr = 0x%X, data = 0x%X, read = 0x%X", rw, addr, data, ret);

  return ret;
}

void dps_write(uint8_t addr, uint16_t data) {
  dps_transaction(0, addr, data);
}

uint16_t dps_read(uint8_t addr) {
  return dps_transaction(1, addr, 0x00);
}

void dps_init() {
  // Set DPS rstb high
  pinMode(PIN_DPS_RESET, OUTPUT);
  digitalWrite(PIN_DPS_RESET, HIGH);

  // Set DPS csb high
  digitalWrite(PIN_DPS_CS, HIGH);

  // Set DPS busy as input (pullup on board?)
  pinMode(PIN_DPS_BUSY, INPUT);

  // TODO Wait until busy goes high. Add timeout
  while(digitalRead(PIN_DPS_BUSY) == LOW) {
    debugD("DPS init: Delaying for busyb");
    delay(1);
  }
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

void setup() {
  // Start wifi in station mode
  WiFi.mode(WIFI_STA);

  // Start serial port
  Serial.begin(115200);

  // Start MDNS
  dns_init();

  // Start remote debug
  debug_init();

  #if defined(ESP8266)
  // Configure timezone and ntp server
  configTime(TZ, NTP_SERVER);

  #elif  defined(ESP32)
  // Set timezone using POSIX string
  setenv("TZ", TZ, 1);  // 1 to overwrite the current value
  tzset();              // Apply the new timezone

  // Configure NTP
  configTime(0, 0, NTP_SERVER);  // Offset is handled by TZ, so set to 0
  #endif

  // Setup wifimanager
  // TODO figure out how to make this non-blocking
  wm.autoConnect("SetupSMU5560");

  // Setup OTA service
  ota_init();

  // Set all csb high to start program
  spi_init();

  #ifdef SPI_DPS
  // Setup DPS SPI
  dps_init();
  #endif

  #ifdef SPI_ADC
  // Setup ADC SPI
  adc_init();
  #endif

  #ifdef SPI_INAMP
  // Setup INAMP SPI
  inamp_init();
  #endif

  //TODO: test/debug stuff
  millis_last = millis();
}


void loop() {
  static uint8_t adc_config = 0;

  // Handle library processing functions
  debug_process();
  ArduinoOTA.handle();  // OTA update

  #ifdef SPI_ADC
  adc_process();
  #endif

  yield();              // ESP processing time


  if (millis() - millis_last > 2000) {
    uint64_t read = 0;

    #ifdef SPI_ADC
    // Read ID register (16 bits) - should read 0x4FDX
    //read = adc_read(0x07, 16);
    //debugD("ADC ID: 0x%X", read);

    // Change ADC input selection
    adc_config++;
    if (adc_config == 1) {
      debugD("ADC: TEMP+ - TEMP-");
      adc_write(0x10, 0x8232,16); // (TEMP+ - TEMP-)
    } else if (adc_config == 2) {
      debugD("ADC: REF+ - REF-");
      adc_write(0x10, 0x82B6,16); // (REF+ - REF-)
    } else {
      debugD("ADC: AIN0 - AIN1");
      adc_write(0x10, 0x8001,16); // (AIN0 - AIN1)
      adc_config = 0;
    }
    #endif

    #ifdef SPI_INAMP
    read = inamp_read(0x2F);
    debugD("Inamp 0x2F: 0x%X", read);
    #endif

    #ifdef SPI_DPS
    read = dps_read(0x3D);
    debugD("DPS 0x3D: 0x%X", read);
    #endif

    millis_last = millis();
  }

}
