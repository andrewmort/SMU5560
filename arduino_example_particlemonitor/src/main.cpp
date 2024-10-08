#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <WiFiManager.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "sps30.h"
#include "ThingSpeak.h"
#include <DNSServer.h>
#include <ESP8266mDNS.h>
#include "RemoteDebug.h"
#include <time.h>

#include "secrets.h"
/*
 * // Contents of secrets.h
 * #define SECRET_OTA_NAME     <ota_name>
 * #define SECRET_OTA_PASS     <ota_password>
 * #define SECRET_CH_ID        <thingspeak_channel>
 * #define SECRET_WRITE_APIKEY <thingspeak_apikey>
 */

#define HOST_NAME "pmonitor"


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
 * ThingSpeak
 ***************************************/
unsigned long myChannelNumber = SECRET_CH_ID;
const char * myWriteAPIKey    = SECRET_WRITE_APIKEY;
WiFiClient  client;

/****************************************
 * OLED display
 ***************************************/
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define DISPLAY_STATE_BLANK 0
#define DISPLAY_STATE_HOME 1
#define DISPLAY_STATE_PART 2
#define DISPLAY_STATE_FAN  3

#define DISPLAY_TIME_REFRESH  (1*1000)  // refresh display every 1 seconds
#define DISPLAY_TIME_BLANK    (   500)  // 0.5 second
#define DISPLAY_TIME_HOME     (5*1000)  // 5 seconds
#define DISPLAY_TIME_PART     (5*1000)  // 5 seconds
#define DISPLAY_TIME_FAN      (5*1000)  // 5 seconds

uint8_t display_seq[] = {DISPLAY_STATE_HOME,
                         DISPLAY_STATE_BLANK,
                         DISPLAY_STATE_PART,
                         DISPLAY_STATE_BLANK,
                         DISPLAY_STATE_FAN,
                         DISPLAY_STATE_BLANK,
                         DISPLAY_STATE_PART,
                         DISPLAY_STATE_BLANK};

#include "FreeSans9pt7b.h"

typedef struct display_info {
  uint32_t millis_next;
  uint32_t millis_last;
  uint8_t state;
  bool invert;
  int8_t idx;
} display_info_t;

display_info_t display_info;


/****************************************
 * SPS30 Particulate senseor
 ***************************************/
#define SPS30_STATE_IDLE 0
#define SPS30_STATE_WAIT 1
#define SPS30_STATE_MEAS 2

#define SPS30_TIME_INIT  (60*1000)          // 1 minute between init attempts
#define SPS30_TIME_CLEAN (7*24*60*60*1000)  // 7 days between cleanings
#define SPS30_TIME_IDLE  (210*1000)         // 3.5 minutes for idle
#define SPS30_TIME_WAIT  (30*1000)          // 30 seconds between idle and measure
#define SPS30_TIME_MEAS  (1*1000)           // 1 second between measurements
#define SPS30_TIME_DEBUG (30*1000)          // 30 second between debug messages
#define SPS30_MEAS_COUNT 60                 // Number of meas to average per cycle

// Time between updates
#define SPS30_TIME_TOTAL (SPS30_TIME_IDLE + SPS30_TIME_WAIT + SPS30_TIME_MEAS * SPS30_MEAS_COUNT)

#define SP30_COMMS Wire

typedef struct particle_info {
  bool init;                    // indicate if sps30 has initialized successfully
  uint8_t state;                // current sps30 state
  uint8_t count;                // measure count
  uint32_t millis_last;         // time of last action
  uint32_t millis_progress;     // time of last idle begin
  uint32_t millis_clean;        // time of last clean
  uint32_t millis_init;         // time of init attempt
  uint32_t millis_debug;        // time of debug message
  float accumulate_mass_pm1p0;  // accumulation of PM 1.0um mass values
  float accumulate_mass_pm2p5;  // accumulation of PM 2.5um mass values
  float accumulate_mass_pm10;   // accumulation of PM 10um mass values
  float accumulate_part_size;   // accumulation of part size values
  float mass_pm1p0;             // final PM 1.0um value
  float mass_pm2p5;             // final PM 2.5um value
  float mass_pm10;              // final PM 10um value
  float part_size;              // final part size value
} particle_info_t;

particle_info_t particle_info;
SPS30 sps30;

/****************************************
 * Fan relay
 ***************************************/

#define RELAY_PIN       12        // GPIO12 - D6

#define RELAY_STATE_OFF 1
#define RELAY_STATE_ON  0

#define RELAY_TIME (30*60*1000)    // 30 minute for timer
#define RELAY_TIME_DEBUG (30*1000) // 30 seconds for debug update

#define RELAY_THRESH_ON  (20.0)
#define RELAY_THRESH_OFF (15.0)

typedef struct relay_info {
  uint32_t millis_last;
  uint32_t millis_debug;
  uint8_t state;
  bool is_countdown;
} relay_info_t;

relay_info_t relay_info;

/****************************************
 *  Remote debug
 ***************************************/
RemoteDebug Debug;

/**********************************************************
 *
 * Relay Functions
 *
 *********************************************************/
void relay_init() {
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_STATE_OFF);
  relay_info.state = RELAY_STATE_OFF;
  relay_info.millis_last = 0;
  relay_info.millis_debug = millis();
  relay_info.is_countdown = false;
}

void relay_process() {
  uint32_t millis_cur;
  millis_cur = millis();

  // Ensure fan is turned on when mass is above threshold
  if (particle_info.mass_pm2p5 > RELAY_THRESH_ON) {
    digitalWrite(RELAY_PIN, RELAY_STATE_ON);
    relay_info.state = RELAY_STATE_ON;
    relay_info.millis_last = millis_cur;
    relay_info.is_countdown = false;
    debugI("Relay: Turn on");
  }

  if (relay_info.state == RELAY_STATE_ON) {
    // Start countdown when crossing threshold
    if (particle_info.mass_pm2p5 < RELAY_THRESH_OFF && !relay_info.is_countdown) {
      relay_info.is_countdown = true;
      relay_info.millis_last = millis_cur;
      debugI("Relay: Start countdown");
    }

    if (relay_info.is_countdown && (millis_cur - relay_info.millis_last > RELAY_TIME)) {
      digitalWrite(RELAY_PIN, RELAY_STATE_OFF);
      relay_info.state = RELAY_STATE_OFF;
      relay_info.millis_last = millis_cur;
      relay_info.is_countdown = false;
      debugI("Relay: Turn off");
    }
  }

  // Print state and measurement progress
  if (millis_cur - relay_info.millis_debug > RELAY_TIME_DEBUG) {
    if (relay_info.state == RELAY_STATE_OFF) {
      debugD("SPS30: state = Off, P2.5 = %fug/m3 < %fug/m3", particle_info.mass_pm2p5, RELAY_THRESH_ON);
    } else {
      if (relay_info.is_countdown) {
        uint32_t time_progress = (millis_cur-relay_info.millis_last)/1000/60;
        uint32_t time_total = RELAY_TIME/1000/60;
        debugD("SPS30: state = On, progress = %um/%um, P2.5 = %fug/m3 < %fug/m3",
          time_progress, time_total, particle_info.mass_pm2p5, RELAY_THRESH_ON);
      } else {
        debugD("SPS30: state = On, P2.5 = %fug/m3 > %fug/m3", particle_info.mass_pm2p5, RELAY_THRESH_OFF);
      }
    }
    particle_info.millis_debug = millis_cur;
  }
}

/**********************************************************
 *
 * OLED Display Functions
 *
 *********************************************************/

void display_init() {
  // Set address for LCD display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  delay(2000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.dim(true);
  display.setTextWrap(false);

  // Initilize display info
  display_info.millis_last = 0;
  display_info.invert = false;
  display_info.idx = -1;
}

// Note: Test display formatting: https://rickkas7.github.io/DisplayGenerator/
void display_process() {
  uint32_t millis_cur;
  millis_cur = millis();

  // Restart display seq if at end
  if (display_info.idx >= sizeof(display_seq)) display_info.idx = 0;

  // Restart display seq if fan not running
  if (display_seq[display_info.idx] == DISPLAY_STATE_FAN
      && relay_info.state == RELAY_STATE_OFF)  display_info.idx = 0;

  display_info.state = display_seq[display_info.idx];

  // Set/refresh screen content
  if (millis_cur - display_info.millis_last > DISPLAY_TIME_REFRESH) {
    uint32_t millis_next = DISPLAY_TIME_BLANK;
    display.clearDisplay();

    if (display_info.state == DISPLAY_STATE_HOME) {
      millis_next = DISPLAY_TIME_HOME;

      display.setCursor(0, 5);

      // Display time and IP address if connected
      if (WiFi.status() == WL_CONNECTED) {
        display.setTextSize(1);
        display.println("");

        // Print time
        time(&now);                     // read the current time
        localtime_r(&now, &timeinfo);         // update the structure tm with the current time
        char buf[64];
        strftime(buf, 64, "%I:%M %p", &timeinfo);
        display.setTextSize(2);
        display.println(buf);

        // Print IP address
        display.setTextSize(1);
        display.println("");
        display.print("IP: ");
        display.println(WiFi.localIP());
        display.println("");
      }

      // Display not connected
      else {
        display.setTextSize(1);
        display.println("");
        display.println("");
        display.println("Not Connected");
      }

    } else if (display_info.state == DISPLAY_STATE_PART) {
      millis_next = DISPLAY_TIME_PART;

      display.setFont(NULL);
      display.setCursor(0, 0);
      display.setTextSize(1);
      display.println("PM2.5:");

      //display.setTextSize(2);
      display.setFont(&FreeSans9pt7b);
      display.setCursor(0,25);
      if (particle_info.init) {
        display.print(" ");
        display.print(particle_info.mass_pm2p5);
        display.println("ug/m3");
      } else {
        display.print(" ");
        display.println("N/A");
      }

      display.setFont(NULL);
      display.setCursor(0,30);
      display.setTextSize(1);
      display.println("");
      display.println("Next reading:");

      //display.setTextSize(2);
      display.setFont(&FreeSans9pt7b);
      display.setCursor(0,60);
      display.print(" ");
      display.print(SPS30_TIME_TOTAL/1000 - (millis_cur - particle_info.millis_progress)/1000);
      display.print("s");

      display.setFont(NULL);

    }else if (display_info.state == DISPLAY_STATE_FAN) {
      millis_next = DISPLAY_TIME_FAN;

      display.setCursor(0, 5);
      display.setTextSize(2);
      display.print("FAN: ");
      if (relay_info.state == RELAY_STATE_ON) {
        display.println("On");
        if (relay_info.is_countdown) {
          uint32_t count_min;
          uint32_t count_sec;

          count_sec = (RELAY_TIME - (millis_cur - relay_info.millis_last))/1000;
          count_min = count_sec/60;
          count_sec = count_sec % 60;

          display.setTextSize(1);
          display.println("");
          display.println("Time remaining:");
          display.setTextSize(2);
          display.print(" ");
          display.print(count_min);
          display.print(":");
          display.print(count_sec);
        }
      } else {
        display.println("Off");
      }
    }

    display_info.millis_last = millis_cur;
    display.display();

    // Go to next state
    if (millis_cur - display_info.millis_next > millis_next) {
      display_info.idx++;
      display_info.millis_next = millis_cur;
    }
  }

}

/**********************************************************
 *
 * SPS30 Particle Sensor Functions
 *
 *********************************************************/
bool particle_init() {
  uint32_t millis_cur;

  debugI("SPS30: Init.");

  // Initialize init state to false
  particle_info.init = false;

  // Initialize times in sps30 info structure
  millis_cur = millis();
  particle_info.millis_last     = 0;
  particle_info.millis_progress = millis_cur;
  particle_info.millis_clean    = millis_cur;
  particle_info.millis_init     = millis_cur;
  particle_info.millis_debug    = millis_cur;

  // Start communication with SPS30
  SP30_COMMS.begin();
  if (sps30.begin(&SP30_COMMS) == false) {
    debugE("SPS30: Could not set I2C communication channel.");
    return particle_info.init;
  }

  // check for SPS30 connection
  if (!sps30.probe()) {
    debugE("SPS30: Could not probe / connect.");
    return particle_info.init;
  } else {
    debugI("SPS30: Detected device.");
  }

  // reset SPS30 connection
  if (!sps30.reset()) {
    debugE("SPS30: Could not reset.");
    return particle_info.init;
  }

  // Turn off auto cleaning
  if (sps30.SetAutoCleanInt(0) != SPS30_ERR_OK) {
    debugE("SPS30: Could not set auto clean interval.");
    return particle_info.init;
  }

  // Mark init successful and set state to idle
  particle_info.init  = true;
  particle_info.state = SPS30_STATE_IDLE;

  return particle_info.init;
}

bool particle_process() {
  uint8_t ret;
  struct sps_values val;
  bool new_reading = false;
  uint32_t millis_cur;

  // Get current time
  millis_cur = millis();

  if (!particle_info.init && millis_cur - particle_info.millis_init > SPS30_TIME_INIT){
    if(!particle_init()) return false;
  }

  // Idle, start measure mode
  if (particle_info.state == SPS30_STATE_IDLE) {
    if (millis_cur - particle_info.millis_last > SPS30_TIME_IDLE) {
      if(!sps30.start()) {
        debugE("SPS30: cannot start measurement.");
        particle_info.init = false;
        return false;
      }

      debugV("SPS30: Start measurement");
      particle_info.state = SPS30_STATE_WAIT;
      particle_info.millis_last = millis_cur;
    }
  }

  // Wait, measure if ready
  else if (particle_info.state == SPS30_STATE_WAIT) {
    // Run clean
    if (millis_cur - particle_info.millis_clean > SPS30_TIME_CLEAN) {
      if (!sps30.clean()) {
        debugE("SPS30: cannot start clean.");
        particle_info.init = false;
        return false;
      }

      particle_info.millis_clean = millis_cur;
      debugV("SPS30: Run clean");
    }

    // Start measurement
    if (millis_cur - particle_info.millis_last > SPS30_TIME_WAIT) {
      particle_info.state = SPS30_STATE_MEAS;
      particle_info.millis_last = millis_cur;
      particle_info.count = 0;
      particle_info.accumulate_mass_pm1p0 = 0;
      particle_info.accumulate_mass_pm2p5 = 0;
      particle_info.accumulate_mass_pm10 = 0;
      particle_info.accumulate_part_size = 0;

      debugV("SPS30: Start data collection.");
    }
  }

  // Measure if ready, idle if ready
  else if (particle_info.state == SPS30_STATE_MEAS) {
    if (millis_cur - particle_info.millis_last > SPS30_TIME_MEAS) {
      ret = sps30.GetValues(&val);

      // Data not ready
      if (ret == SPS30_ERR_DATALENGTH) {
        debugI("SPS30: data may not be ready.");
      }

      // Data ready, update next measured value
      else if (ret == SPS30_ERR_OK) {
        particle_info.accumulate_mass_pm1p0 += val.MassPM1;
        particle_info.accumulate_mass_pm2p5 += val.MassPM2;
        particle_info.accumulate_mass_pm10  += val.MassPM10;
        particle_info.accumulate_part_size  += val.PartSize;
        particle_info.count++;
        particle_info.millis_last = millis_cur;

        debugV("SPS30: Next measurement (%d/%d): P2.5 = %fug/m3",
          particle_info.count, SPS30_MEAS_COUNT, val.MassPM2);
      }

      // Error
      else {
        debugE("SPS30: cannot start clean.");
        particle_info.init = false;
        return false;
      }
    }

    // Idle if finished all measurements
    if (particle_info.count >= SPS30_MEAS_COUNT) {
      particle_info.mass_pm1p0 = particle_info.accumulate_mass_pm1p0/particle_info.count;
      particle_info.mass_pm2p5 = particle_info.accumulate_mass_pm2p5/particle_info.count;
      particle_info.mass_pm10  = particle_info.accumulate_mass_pm10/particle_info.count;
      particle_info.part_size  = particle_info.accumulate_part_size/particle_info.count;
      new_reading = true;

      debugI("SPS30: Finished measurement.");
      debugI("SPS30:  P1.0 = %fug/m3", particle_info.mass_pm1p0);
      debugI("SPS30:  P2.5 = %fug/m3", particle_info.mass_pm2p5);
      debugI("SPS30:  P10  = %fug/m3", particle_info.mass_pm10);
      debugI("SPS30:  Part Size = %fum", particle_info.part_size);

      // Put sensor back to idle
      if(!sps30.stop()) {
        debugE("SPS30: cannot stop measurement.");
        particle_info.init = false;
        return false;
      }

      particle_info.state = SPS30_STATE_IDLE;
      particle_info.millis_last = millis_cur;
      particle_info.millis_progress = millis_cur;
    }
  }

  // Print state and measurement progress
  if (millis_cur - particle_info.millis_debug > SPS30_TIME_DEBUG) {
    String str_state;
    uint32_t time_progress, time_total;

    // Get state as string
    if (particle_info.state == SPS30_STATE_IDLE) str_state = "Idle";
    else if (particle_info.state == SPS30_STATE_WAIT) str_state = "Wait";
    else if (particle_info.state == SPS30_STATE_MEAS) str_state = "Meas";

    time_progress = (millis_cur - particle_info.millis_progress)/1000;
    time_total = SPS30_TIME_TOTAL/1000;

    debugD("SPS30: state = %s, progress = %us/%us", str_state, time_progress, time_total);
    particle_info.millis_debug = millis_cur;
  }

  return new_reading;
}

/**********************************************************
 *
 * ThingSpeak Functions
 *
 *********************************************************/
void thingspeak_init() {
  // Initialize ThingSpeak
  ThingSpeak.begin(client);
}

void thingspeak_update() {
  ThingSpeak.setField(1, particle_info.mass_pm2p5);
  ThingSpeak.setField(2, particle_info.part_size);
  ThingSpeak.setField(3, particle_info.mass_pm1p0);
  ThingSpeak.setField(4, particle_info.mass_pm10);
  ThingSpeak.setField(5, relay_info.state);
  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  if(x == 200){
    debugV("Channel update successful.\n");
  }
  else{
    debugE("Problem updating channel. HTTP error code %s\n", String(x).c_str());
  }

}

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


/******************************************************************************
 *
 *  Main setup and loop functions
 *
 *****************************************************************************/

void setup() {
  // Start wifi in station mode
  WiFi.mode(WIFI_STA);

  // Start serial port
  Serial.begin(115200);

  // Initialize relays
  relay_init();

  // Start MDNS
  mdns_init();

  // Start remove debug
  debug_init();

  // Configure timezone and ntp server
  configTime(TZ, NTP_SERVER);

  // Setup wifimanager in non-blocking mode
//  wm.setConfigPortalBlocking(false);
  wm.autoConnect("SetupPMonitor");

  // Setup OTA service
  ota_init();

  // Start OLED display
  display_init();

  // Start particulate sensor
  particle_init();

  // Start ThingSpeak
  thingspeak_init();
}

void loop() {
  // Handle library processing functions
  Debug.handle();       // remote debug
//  wm.process();         // wifi manager
  ArduinoOTA.handle();  // OTA update
  yield();              // ESP processing time

  // process particle sensor
  if (particle_process()) {
    // update thingspeak data if new sensor reading
    thingspeak_update();
  }

  // process relay settings
  relay_process();

  // process display update
  display_process();

}
