#include "Adafruit_FONA.h" // https://github.com/botletics/SIM7000-LTE-Shield/tree/master/Code
#include <TimeLib.h>

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

// Define *one* of the following lines:
//#define SIMCOM_2G // SIM800/808/900/908, etc.
//#define SIMCOM_3G // SIM5320
#define SIMCOM_7000
//#define SIMCOM_7070
//#define SIMCOM_7500
//#define SIMCOM_7600

// Uncomment *one* of the following protocols you want to use
// to send data to the cloud! Leave the other commented out
//#define PROTOCOL_HTTP_GET         // Generic
 #define PROTOCOL_HTTP_POST        // Generic
/************************* PIN DEFINITIONS *********************************/
// For botletics SIM7000 shield
#define FONA_PWRKEY 4
#define FONA_RST -1
//#define FONA_DTR 8 // Connect with solder jumper
//#define FONA_RI 9 // Need to enable via AT commands
#define FONA_TX 12 // Microcontroller RX
#define FONA_RX 13 // Microcontroller TX
//#define T_ALERT 12 // Connect with solder jumper

#define GPS_ON
#define TEMP_ON
// #define DUMMY_ON
#define IMG_ON
// #define NVR_END

#include "SoftwareSerial.h"
SoftwareSerial fonaSS(FONA_TX, FONA_RX);

SoftwareSerial *fonaSerial = &fonaSS;

// Use this one for LTE CAT-M/NB-IoT modules (like SIM7000)
// Notice how we don't include the reset pin because it's reserved for emergencies on the LTE module!
#if defined(SIMCOM_7000) || defined(SIMCOM_7070) || defined(SIMCOM_7500) || defined(SIMCOM_7600)
Adafruit_FONA_LTE fona = Adafruit_FONA_LTE();
#endif


/****************************** OTHER STUFF ***************************************/
// For sleeping the AVR
//#include <avr/sleep.h>
//#include <avr/power.h>

// The following line is used for applications that require repeated data posting, like GPS trackers
// Comment it out if you only want it to post once, not repeatedly every so often
#define samplingRate 10 // The time in between posts, in seconds

// The following line can be used to turn off the shield after posting data. This
// could be useful for saving energy for sparse readings but keep in mind that it
// will take longer to get a fix on location after turning back on than if it had
// already been on. Comment out to leave the shield on after it posts data.
// #define turnOffShield // Turn off shield after posting data

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
char imei[16] = {0}; // Use this for device ID
uint8_t type;
uint16_t battLevel = 0; // Battery level (percentage)
float latitude, longitude, speed_kph, heading, altitude, sec;
uint16_t yr;
uint8_t mth, dy, hr, mint;
bool tmpSuccess = false;
int imgFail = 0;
//char PIN[5] = "1234"; // SIM card PIN

// char URL[200];  // Make sure this is long enough for your request URL
char URL_IMG[200];  // Make sure this is long enough for your request URL
char URL_TMP[200];  // Make sure this is long enough for your request URL
char URL_HUM[200];  // Make sure this is long enough for your request URL
char body[100]; // Make sure this is long enough for POST body
char bodyTMP[200]; // Make sure this is long enough for POST body
char latBuff[12], longBuff[12], locBuff[50], speedBuff[12], imgFailBuff[3],
     headBuff[12], altBuff[12], tempBuff[12], humidBuff[12], battBuff[12], tsBuff[20];
unsigned long long ts;


// Include Required Libraries

// Camera libraries
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_io.h"

// MicroSD Libraries
#include "FS.h"
#include "SD_MMC.h"

// EEPROM Library
#include "EEPROM.h"

// extern "C" {
#include "base64.h"
// }

// Use 1 byte of EEPROM space
#define EEPROM_SIZE 1

// Counter for picture number
unsigned int pictureCount = 0;

// Pin definitions for CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22


// -----------DHT22 SECTION------------
#include "DHT.h"

#define DHTPIN 33

#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);
// -----------DHT22 SECTION------------
void configESPCamera() {
  // Configure Camera parameters

  // Object to store the camera configuration parameters
  camera_config_t config;

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; // Choices are YUV422, GRAYSCALE, RGB565, JPEG

  // Select lower framesize if the camera doesn't support PSRAM
  if (psramFound()) {
    config.frame_size = FRAMESIZE_VGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 10; //10-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // Initialize the Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  // Camera quality adjustments
  sensor_t * s = esp_camera_sensor_get();

  // BRIGHTNESS (-2 to 2)
  s->set_brightness(s, 0);
  // CONTRAST (-2 to 2)
  s->set_contrast(s, 0);
  // SATURATION (-2 to 2)
  s->set_saturation(s, 0);
  // SPECIAL EFFECTS (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  s->set_special_effect(s, 0);
  // WHITE BALANCE (0 = Disable , 1 = Enable)
  s->set_whitebal(s, 1);
  // AWB GAIN (0 = Disable , 1 = Enable)
  s->set_awb_gain(s, 1);
  // WB MODES (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  s->set_wb_mode(s, 0);
  // EXPOSURE CONTROLS (0 = Disable , 1 = Enable)
  s->set_exposure_ctrl(s, 1);
  // AEC2 (0 = Disable , 1 = Enable)
  s->set_aec2(s, 0);
  // AE LEVELS (-2 to 2)
  s->set_ae_level(s, 0);
  // AEC VALUES (0 to 1200)
  s->set_aec_value(s, 300);
  // GAIN CONTROLS (0 = Disable , 1 = Enable)
  s->set_gain_ctrl(s, 1);
  // AGC GAIN (0 to 30)
  s->set_agc_gain(s, 0);
  // GAIN CEILING (0 to 6)
  s->set_gainceiling(s, (gainceiling_t)0);
  // BPC (0 = Disable , 1 = Enable)
  s->set_bpc(s, 0);
  // WPC (0 = Disable , 1 = Enable)
  s->set_wpc(s, 1);
  // RAW GMA (0 = Disable , 1 = Enable)
  s->set_raw_gma(s, 1);
  // LENC (0 = Disable , 1 = Enable)
  s->set_lenc(s, 1);
  // HORIZ MIRROR (0 = Disable , 1 = Enable)
  s->set_hmirror(s, 0);
  // VERT FLIP (0 = Disable , 1 = Enable)
  s->set_vflip(s, 0);
  // DCW (0 = Disable , 1 = Enable)
  s->set_dcw(s, 1);
  // COLOR BAR PATTERN (0 = Disable , 1 = Enable)
  s->set_colorbar(s, 0);

}

void initMicroSDCard() {
  // Start the MicroSD card

  Serial.println("Mounting MicroSD Card");
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("MicroSD Card Mount Failed");
    return;
  }
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No MicroSD Card found");
    return;
  }

}

void takeNewPhoto(String path, String& output) {
  // Take Picture with Camera

  // Setup frame buffer
  camera_fb_t  * fb = esp_camera_fb_get();

  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  // Save picture to microSD card
  fs::FS &fs = SD_MMC;
  File file = fs.open(path.c_str(), FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file in write mode");
  }
  else {
    file.write(fb->buf, fb->len); // payload (image), payload length
    Serial.printf("Saved file to path: %s\n", path.c_str());
  }
  // Close the file
  file.close();
//  Convert picture to binary
  // fileSize = fb->len;
  output = base64::encode(fb->buf, fb->len);
  // fileinput = (char*)malloc(fileSize + 1);
  // memcpy((void*)fileinput, (void*) fb->buf, fileSize);
  // fileinput[fileSize] = '\0';
//  SD_MMC.end();
//  char* binImage = encode(fileinput, fileSize + 1);
//  for(int i = 0; i < fileSize; i++)
//{
//  Serial.println(binImage[i]);
//}
  // Return the frame buffer back to the driver for reuse
  esp_camera_fb_return(fb);
}

boolean initFONA(){
  // digitalWrite(FONA_PWRKEY, HIGH); // Default state
  // delay(300);
  // digitalWrite(FONA_PWRKEY, LOW); // Default state
  Serial.println(F("Turning on module"));


  // fona.powerOn(FONA_PWRKEY); // Power on the module
  Serial.println(F("Setting up module"));
  if (!moduleSetup()) // Establishes first-time serial comm and prints IMEI
  {
    return false;
  }
 
  // Unlock SIM card if needed
  // Remember to uncomment the "PIN" variable definition above
  /*
  if (!fona.unlockSIM(PIN)) {
    Serial.println(F("Failed to unlock SIM card"));
  }
  */

  // Set modem to full functionality
  fona.setFunctionality(1); // AT+CFUN=1

  fona.setNetworkSettings(F("hicard")); // for Singtel Pre-Paid

  // Optionally configure HTTP gets to follow redirects over SSL.
  // Default is not to follow SSL redirects, however if you uncomment
  // the following line then redirects over SSL will be followed.
  //fona.setHTTPSRedirect(true);

  /*
  // Other examples of some things you can set:
  fona.setPreferredMode(38); // Use LTE only, not 2G
  fona.setPreferredLTEMode(1); // Use LTE CAT-M only, not NB-IoT
  fona.setOperatingBand("CAT-M", 12); // AT&T uses band 12
//  fona.setOperatingBand("CAT-M", 13); // Verizon uses band 13
  fona.enableRTC(true);
  
  fona.enableSleepMode(true);
  fona.set_eDRX(1, 4, "0010");
  fona.enablePSM(true);

  // Set the network status LED blinking pattern while connected to a network (see AT+SLEDS command)
  fona.setNetLED(true, 2, 64, 3000); // on/off, mode, timer_on, timer_off
  fona.setNetLED(false); // Disable network status LED
  */

  // Perform first-time GPS/GPRS setup if the shield is going to remain on,
  // otherwise these won't be enabled in loop() and it won't work!
#ifndef turnOffShield
  #ifdef GPS_ON
  // Enable GPS
  while (!fona.enableGPS(true)) {
    Serial.println(F("Failed to turn on GPS, retrying..."));
    delay(2000); // Retry every 2s
  }
  Serial.println(F("Turned on GPS!"));
  #endif

  #if !defined(SIMCOM_3G) && !defined(SIMCOM_7500) && !defined(SIMCOM_7600)
    // Disable GPRS just to make sure it was actually off so that we can turn it on
    // if (!fona.enableGPRS(false)) Serial.println(F("Failed to disable GPRS!"));
    
    // Turn on GPRS
    int counter = 0;
    while (!fona.enableGPRS(true)) {
      Serial.println(F("Failed to enable GPRS, retrying..."));
      delay(2000); // Retry every 2s
      if(counter == 20){
        return initFONA();
      }
    }
    Serial.println(F("Enabled GPRS!"));
  #endif
#endif

  return true;
}

void setup() {
  // Start Serial Monitor
  Serial.begin(115200); // this is only the baud rate for monitoring not for communicating fona module since the fona module use software serial not hardware serial
  Serial.println(F("*** SIMCom Module IoT Example ***"));
  
  pinMode(13, OUTPUT); // set to output but later fona will set back to input
  digitalWrite(13, HIGH); // when initializing SD Card Mount, this will prevent entering SPI Mode
  delay(10);
  // Initialize the MicroSD
  Serial.print("Initializing the MicroSD card module... ");
  initMicroSDCard();

  // Initialize DHT22(Temp)
  dht.begin();

  // Initialize the camera
  Serial.print("Initializing the camera module...");
  configESPCamera();
  Serial.println("Camera OK!");


  #ifdef LED
    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);
  #endif
  
  fonaSS.begin(38400); // baudrate for the software serial
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, 1);
  pinMode(FONA_RST, OUTPUT);
  // pinMode(FONA_PWRKEY, INPUT);
  digitalWrite(FONA_RST, HIGH); // Default state
  if (!initFONA())
  {
    return;
  }

}

int countFail = 0;

void loop() {
    // ----------------DHT22 LOOP-------------------
  // Wait a few seconds between measurements.
  // delay(2000);

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  // Compute heat index in Celsius (isFahreheit = false)
  // float hic = dht.computeHeatIndex(t, h, false);
  // ----------------DHT22 LOOP-------------------

  // Connect to cell network and verify connection
  // If unsuccessful, keep retrying every 2s until a connection is made
  int counter = 0;
  while (!netStatus()) {
    Serial.println(F("Failed to connect to cell network, retrying..."));
    delay(2000); // Retry every 2s
    if(counter == 10){
      initFONA();
      return;
    }
  }
  Serial.println(F("Connected to cell network!"));

  // Measure battery level
  // Note: on the LTE shield this won't be accurate because the SIM7000
  // is supplied by a regulated 3.6V, not directly from the battery. You
  // can use the Arduino and a voltage divider to measure the battery voltage
  // and use that instead, but for now we will use the function below
  // only for testing.
  battLevel = readVcc(); // Get voltage in mV

  delay(500); // I found that this helps

  // Turn on GPS if it wasn't on already (e.g., if the module wasn't turned off)
#ifdef turnOffShield
  while (!fona.enableGPS(true)) {
    Serial.println(F("Failed to turn on GPS, retrying..."));
    delay(2000); // Retry every 2s
  }
  Serial.println(F("Turned on GPS!"));
#endif

  // Get a fix on location, try every 2s
  // Use the top line if you want to parse UTC time data as well, the line below it if you don't care
  
  #ifdef GPS_ON
  while (!fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude, &yr, &mth, &dy, &hr, &mint, &sec)) {
//  while (!fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude)) {
    Serial.println(F("Failed to get GPS location, retrying..."));
    delay(2000); // Retry every 2s
  }
  
  Serial.println(F("Found 'eeeeem!"));
  Serial.println(F("---------------------"));
  Serial.print(F("Latitude: ")); Serial.println(latitude, 6);
  Serial.print(F("Longitude: ")); Serial.println(longitude, 6);
  Serial.print(F("Speed: ")); Serial.println(speed_kph);
  Serial.print(F("Heading: ")); Serial.println(heading);
  Serial.print(F("Altitude: ")); Serial.println(altitude);
  #endif
 
  // /*
  // Uncomment this if you care about parsing UTC time
  Serial.print(F("Year: ")); Serial.println(yr);
  Serial.print(F("Month: ")); Serial.println(mth);
  Serial.print(F("Day: ")); Serial.println(dy);
  Serial.print(F("Hour: ")); Serial.println(hr);
  Serial.print(F("Minute: ")); Serial.println(mint);
  Serial.print(F("Second: ")); Serial.println(sec);
  // */
  Serial.println(F("---------------------"));
  
  // If the shield was already on, no need to re-enable
#if defined(turnOffShield) && !defined(SIMCOM_3G) && !defined(SIMCOM_7500) && !defined(SIMCOM_7600)
  // Disable GPRS just to make sure it was actually off so that we can turn it on
  if (!fona.enableGPRS(false)) Serial.println(F("Failed to disable GPRS!"));
  
  // Turn on GPRS
  while (!fona.enableGPRS(true)) {
    Serial.println(F("Failed to enable GPRS, retrying..."));
    delay(2000); // Retry every 2s
  }
  Serial.println(F("Enabled GPRS!"));
#endif

  // Post something like temperature and battery level to the web API
  // Construct URL and post the data to the web API
  #ifndef GPS_ON
  latitude = 1.37862;
  longitude = 103.76389;
  #endif
  #ifdef GPS_ON
  setTime(hr, mint, sec, dy, mth, yr);
  ts = now();
  #endif

  /*--------------CAMERA SETUP SECTION--------------*/
  // Disable brownout detector
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);



  // initialize EEPROM with predefined size
//  EEPROM.begin(EEPROM_SIZE);
//  pictureCount = EEPROM.read(0) + 1;

  // Path where new picture will be saved in SD Card
  String path = "/image" + String(ts) + ".jpg";
  Serial.printf("Picture file name: %s\n", path.c_str());

  // Take and Save Photo
  // char *fileinput;
  // unsigned int fileSize;
  String picdataBase64;
  takeNewPhoto(path, picdataBase64);
  // std::string picInString = bufferToString(fileinput, fileSize);
  // Update EEPROM picture number counter
//  EEPROM.write(0, pictureCount);
//  EEPROM.commit();
/*-------------- END CAMERA SETUP SECTION--------------*/

  // Format the floating point numbers
  dtostrf(latitude, 1, 6, latBuff);
  dtostrf(longitude, 1, 6, longBuff);
  dtostrf(speed_kph, 1, 0, speedBuff);
  dtostrf(heading, 1, 0, headBuff);
  dtostrf(altitude, 1, 1, altBuff);
  dtostrf(t, 2, 2, tempBuff);
  dtostrf(h, 2, 2, humidBuff);
//  dtostrf(lattest, 1, 6, latBuff);
//  dtostrf(longtest, 1, 6, longBuff);
//  dtostrf(0.5, 1, 0, speedBuff);
//  dtostrf(71.71, 1, 0, headBuff);
//  dtostrf(5.72, 1, 1, altBuff);
  
//  dtostrf(temperature, 1, 2, tempBuff); // float_val, min_width, digits_after_decimal, char_buffer
  dtostrf(battLevel, 1, 0, battBuff);

  // Also construct a combined, comma-separated location array
  // (many platforms require this for dashboards, like Adafruit IO):
  sprintf(locBuff, "%s,%s,%s,%s", speedBuff, latBuff, longBuff, altBuff); // This could look like "10,33.123456,-85.123456,120.5"
  
  // Construct the appropriate URL's and body, depending on request type
  // In this example we use the IMEI as device ID

#ifdef PROTOCOL_HTTP_GET
  // GET request
  
  counter = 0; // This counts the number of failed attempts tries
  
  #if defined(SIMCOM_3G) || defined(SIMCOM_7500) || defined(SIMCOM_7600)
    // You can adjust the contents of the request if you don't need certain things like speed, altitude, etc.
    sprintf(URL, "GET /dweet/for/%s?lat=%s&long=%s&speed=%s&head=%s&alt=%s&temp=%s&batt=%s HTTP/1.1\r\nHost: dweet.io\r\n\r\n",
            imei, latBuff, longBuff, speedBuff, headBuff, altBuff, tempBuff, battBuff);
            
    // Try a total of three times if the post was unsuccessful (try additional 2 times)
    while (counter < 3 && !fona.postData("www.dweet.io", 443, "HTTPS", URL)) { // Server, port, connection type, URL
      Serial.println(F("Failed to complete HTTP/HTTPS request..."));
      counter++; // Increment counter
      delay(1000);
    }
  #else
    sprintf(URL, "http://dweet.io/dweet/for/%s?lat=%s&long=%s&speed=%s&head=%s&alt=%s&temp=%s&batt=%s", imei, latBuff, longBuff,
            speedBuff, headBuff, altBuff, tempBuff, battBuff);
          
    while (counter < 3 && !fona.postData("GET", URL)) {
      Serial.println(F("Failed to post data, retrying..."));
      counter++; // Increment counter
      delay(1000);
    }
  #endif
  
#elif defined(PROTOCOL_HTTP_POST)  
  // You can also do a POST request instead

  #if defined(SIMCOM_3G) || defined(SIMCOM_7500) || defined(SIMCOM_7600)
    sprintf(body, "{\"lat\":%s,\"long\":%s}\r\n", latBuff, longBuff, speedBuff, headBuff, altBuff); // Terminate with CR+NL
    sprintf(URL, "POST /dweet/for/%s HTTP/1.1\r\nHost: dweet.io\r\nContent-Length: %i\r\n\r\n", imei, strlen(body));

    while (counter < 3 && !fona.postData("www.dweet.io", 443, "HTTPS", URL, body)) { // Server, port, connection type, URL
      Serial.println(F("Failed to complete HTTP/HTTPS request..."));
      counter++; // Increment counter
      delay(1000);
    }
  #else
    // sprintf(URL, "http://dweet.io/dweet/for/%s", imei);
//    sprintf(body, "{\"lat\":%s,\"long\":%s}", latBuff, longBuff);
    // sprintf(body, "{\"pic\":%s}", picdataBase64.c_str());
    unsigned char success = true;
    String payload("{\"pic\":\"");
    success &= payload.concat(picdataBase64);
    success &= payload.concat("\"}");

    // Let's try a POST request to thingsboard.io
    // Please note this can me memory-intensive for the Arduino Uno
    // and may not work. You might have to split it up into a couple requests
    // and send part of the data in one request, and the rest in the other, etc.
    // Perhaps an easier solution is to swap out the Uno with an Arduino Mega.
    
    const char * tokenIMG = "Cam"; // From thingsboard.io device
    const char * tokenTMP = "TempPass1"; // From thingsboard.io device
    sprintf(URL_IMG, "http://thingsboard.cloud/api/v1/%s/telemetry", tokenIMG);
    sprintf(URL_TMP, "http://thingsboard.cloud/api/v1/%s/telemetry", tokenTMP);
    // sprintf(body, "{\"lat\":%s,\"long\":%s,\"speed\":%s,\"head\":%s,\"alt\":%s,\"temp\":%s,\"batt\":%s}", latBuff, longBuff,
    //         speedBuff, headBuff, altBuff, tempBuff, battBuff);
  //  sprintf(body, "{\"lat\":%s,\"long\":%s}", latBuff, longBuff); // If all you want is lat/long
    // sprintf(bodyTMP, "{\"temperature\":%s}", tempBuff);
    #ifdef GPS_ON
    Serial.println(ts);
    Serial.println(ts*1000);
    sprintf(bodyTMP, "{\"ts\":%llu,\"values\":{\"temperature\":%s,\"humidity\":%s,\"long\":%s,\"lat\":%s}}", ts*1000, tempBuff, humidBuff, longBuff, latBuff); // transmit timestamp with payload
    #endif
    #ifndef GPS_ON
    sprintf(bodyTMP, "{\"temperature\":%s,\"humidity\":%s,\"long\":%s,\"lat\":%s}", tempBuff, humidBuff, longBuff, latBuff);
    #endif

    dtostrf(imgFail, 1, 0, imgFailBuff);
    sprintf(body, "{\"imgfail\":%s}", imgFailBuff);

    #ifdef DUMMY_ON
    // POST dummy payload to encourage successful image POST
    if (fona.postData("POST", URL_IMG, body, 5000)) {
      Serial.println(F("Succeeded to POST dummy payload..."));
    }
    else{
      Serial.println(F("Failed to POST dummy payload..."));
      imgFail++;
      delay(200);
      return;
    }
    #endif

    #ifdef TEMP_ON
    // POST temp, humidity, GPS data
    if (!tmpSuccess){
      if (fona.postData("POST", URL_TMP, bodyTMP, 5000)) {
        Serial.println(F("Succeeded to complete HTTP POST..."));
        countFail = 0;
        // tmpSuccess = true;
      }
      else{
        Serial.println(F("Failed to complete HTTP POST..."));
        delay(200);
        countFail++;
        if(countFail == 5){
          initFONA();
          countFail = 0;
        }
        return;
      }
    }
    #endif

    #ifdef IMG_ON
    // POST image data
    if (fona.postData("POST", URL_IMG, payload.c_str(), 5000)) {
      Serial.println(F("Succeeded to complete HTTP POST..."));
      countFail = 0;
    }
    else{
      Serial.println(F("Failed to complete HTTP POST..."));
      delay(200);
      countFail++;
      if(countFail == 5){
        initFONA();
        countFail = 0;
      }
      return;
    }
    #endif


        //Only run the code below if you want to turn off the shield after posting data
      #ifdef turnOffShield
        // Disable GPRS
        // Note that you might not want to check if this was successful, but just run it
        // since the next command is to turn off the module anyway
        if (!fona.enableGPRS(false)) Serial.println(F("Failed to disable GPRS!"));

        // Turn off GPS
      //  if (!fona.enableGPS(false)) Serial.println(F("Failed to turn off GPS!"));
        
        // Power off the module. Note that you could instead put it in minimum functionality mode
        // instead of completely turning it off. Experiment different ways depending on your application!
        // You should see the "PWR" LED turn off after this command
       if (!fona.powerDown()) Serial.println(F("Failed to power down FONA!")); // No retries
        counter = 0;
        while (counter < 3 && !fona.powerDown()) { // Try shutting down 
          Serial.println(F("Failed to power down FONA!"));
          counter++; // Increment counter
          delay(1000);
        }
      #endif
    
    // #ifdef GPS_ON
    // Turn off GPS
    //  if (!fona.enableGPS(false)) Serial.println(F("Failed to turn off GPS!"));
    // #endif
    // Disable GPRS
    // if (!fona.enableGPRS(false)) Serial.println(F("Failed to disable GPRS!"));
    // Serial.println("GPRS Turned Off");
    // Serial.println("Turning off FONA");
      // Alternative to the AT command method above:
      // If your FONA has a PWRKEY pin connected to your MCU, you can pulse PWRKEY
      // LOW for a little bit, then pull it back HIGH, like this:
    // digitalWrite(FONA_PWRKEY, HIGH);
    //  delay(600); // Minimum of 64ms to turn on and 500ms to turn off for FONA 3G. Check spec sheet for other types
    // delay(1300); // Minimum of 1.2s for SIM7000
    // digitalWrite(FONA_PWRKEY, HIGH);
    #ifdef NVR_END
    return;
    #endif
    
    Serial.println("Turning off ESP");
    delay(100);
    esp_light_sleep_start();
    return;
  #endif
#endif

  
  
  // Shut down the MCU to save power
#ifndef samplingRate
  Serial.println(F("Shutting down..."));
  delay(5); // This is just to read the response of the last AT command before shutting down
  MCU_powerDown(); // You could also write your own function to make it sleep for a certain duration instead
#else
  // The following lines are for if you want to periodically post data (like GPS tracker)
  // Serial.print(F("Waiting for ")); Serial.print(samplingRate); Serial.println(F(" seconds\r\n"));
  // delay(samplingRate * 1000UL); // Delay

  // Only run the initialization again if the module was powered off
  // since it resets back to 115200 baud instead of 4800.
  #ifdef turnOffShield
    fona.powerOn(FONA_PWRKEY); // Powers on the module if it was off previously
    moduleSetup();
  #endif
    
#endif
}

boolean moduleSetup() {
  // SIM7000 takes about 3s to turn on and SIM7500 takes about 15s
  // Press Arduino reset button if the module is still turning on and the board doesn't find it.
  // When the module is on it should communicate right after pressing reset

  // Software serial:
//  fonaSS.begin(115200); // Default SIM7000 shield baud rate

//  Serial.println(F("Configuring to 9600 baud"));
//  fonaSS.println("AT+IPR=9600"); // Set baud rate
  delay(100); // Short pause to let the command run
  if (! fona.begin(fonaSS)) {
    Serial.println(F("Couldn't find FONA"));
    return false; // Don't proceed if it couldn't find the device
  }

  // Hardware serial:
  /*
  fonaSerial->begin(115200); // Default SIM7000 baud rate

  if (! fona.begin(*fonaSerial)) {
    DEBUG_PRINTLN(F("Couldn't find SIM7000"));
  }
  */
  
  // The commented block of code below is an alternative that will find the module at 115200
  // Then switch it to 9600 without having to wait for the module to turn on and manually
  // press the reset button in order to establish communication. However, once the baud is set
  // this method will be much slower.
  /*
  fonaSerial->begin(115200); // Default LTE shield baud rate
  fona.begin(*fonaSerial); // Don't use if statement because an OK reply could be sent incorrectly at 115200 baud

  Serial.println(F("Configuring to 9600 baud"));
  fona.setBaudrate(9600); // Set to 9600 baud
  fonaSerial->begin(9600);
  if (!fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find modem"));
    while(1); // Don't proceed if it couldn't find the device
  }
  */

  type = fona.type();
  Serial.println(F("FONA is OK"));
  Serial.print(F("Found "));
  switch (type) {
    case SIM800L:
      Serial.println(F("SIM800L")); break;
    case SIM800H:
      Serial.println(F("SIM800H")); break;
    case SIM808_V1:
      Serial.println(F("SIM808 (v1)")); break;
    case SIM808_V2:
      Serial.println(F("SIM808 (v2)")); break;
    case SIM5320A:
      Serial.println(F("SIM5320A (American)")); break;
    case SIM5320E:
      Serial.println(F("SIM5320E (European)")); break;
    case SIM7000:
      Serial.println(F("SIM7000")); break;
    case SIM7070:
      Serial.println(F("SIM7070")); break;
    case SIM7500:
      Serial.println(F("SIM7500")); break;
    case SIM7600:
      Serial.println(F("SIM7600")); break;
    default:
      Serial.println(F("???")); break;
  }
  
  // Print module IMEI number.
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("Module IMEI: "); Serial.println(imei);
  }

  return true;
}

// Read the module's power supply voltage
float readVcc() {
  // Read battery voltage
  if (!fona.getBattVoltage(&battLevel)) Serial.println(F("Failed to read batt"));
  else Serial.print(F("battery = ")); Serial.print(battLevel); Serial.println(F(" mV"));

  // Read LiPo battery percentage
//  if (!fona.getBattPercent(&battLevel)) Serial.println(F("Failed to read batt"));
//  else Serial.print(F("BAT % = ")); Serial.print(battLevel); Serial.println(F("%"));

  return battLevel;
}

bool netStatus() {
  int n = fona.getNetworkStatus();
  
  Serial.print(F("Network status ")); Serial.print(n); Serial.print(F(": "));
  if (n == 0) Serial.println(F("Not registered"));
  if (n == 1) Serial.println(F("Registered (home)"));
  if (n == 2) Serial.println(F("Not registered (searching)"));
  if (n == 3) Serial.println(F("Denied"));
  if (n == 4) Serial.println(F("Unknown"));
  if (n == 5) Serial.println(F("Registered roaming"));

  if (!(n == 1 || n == 5)) return false;
  else return true;
}

// Turn off the MCU completely. Can only wake up from RESET button
// However, this can be altered to wake up via a pin change interrupt
void MCU_powerDown() {
//  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
//  ADCSRA = 0; // Turn off ADC
//  power_all_disable ();  // Power off ADC, Timer 0 and 1, serial interface
//  sleep_enable();
//  sleep_cpu();
}
