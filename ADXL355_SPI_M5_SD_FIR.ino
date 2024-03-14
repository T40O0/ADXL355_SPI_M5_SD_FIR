#include <SD.h>
#include <M5Unified.h>
#include <M5_ADXL355.h>

//==============================================================================
// for RTC
// M5Unified Sample
// https://github.com/m5stack/M5Unified/blob/master/examples/Basic/Rtc/Rtc.ino

#if defined ( ARDUINO )

#define WIFI_SSID     "your id"
#define WIFI_PASSWORD "your pass"
#define NTP_TIMEZONE  "your zone"
#define NTP_SERVER1   "your server1"
#define NTP_SERVER2   "your server2"
#define NTP_SERVER3   "your server3"

#include <WiFi.h>

// Different versions of the framework have different SNTP header file names and availability.
#if __has_include (<esp_sntp.h>)
  #include <esp_sntp.h>
  #define SNTP_ENABLED 1
#elif __has_include (<sntp.h>)
  #include <sntp.h>
  #define SNTP_ENABLED 1
#endif

#endif

#ifndef SNTP_ENABLED
#define SNTP_ENABLED 0
#endif

//==============================================================================

unsigned int hz = 100; 
unsigned int dtWrite = 1000 / hz; //delta T msec
unsigned int SDWriteTime = 15; //sec

auto dt = M5.Rtc.getDateTime();
char filePath[10];
char fileName[30];
int fileDate = 0;
int fileDateTime;
File f;

const String accHeader = "Time(msec), x(cm/s2), y(cm/s2), z(cm/s2)";
double AccX = 0.;
double AccY = 0.;
double AccZ = 0.;
double AccFirX = 0.;
double AccFirY = 0.;
double AccFirZ = 0.;
const int BufNum = 201; // array num, for FIR filter
double RingBufX[BufNum] = {}; //initialize with 0
double RingBufY[BufNum] = {};
double RingBufZ[BufNum] = {};
unsigned int BufIndex = 0;

//==============================================================================

// Create an instance of ADXL355
//PL::ADXL355(SCK, MISO, MOSI, SS)
PL::ADXL355 adxl355(26, 36, 32, 33);
auto range = PL::ADXL355_Range::range2g;// ADXL355 range: +/- 2 g

//auto ODR = PL::ADXL355_OutputDataRate::odr4000; // 4000 Hz (low-pass filter: -3.5dB at 1000 Hz)
auto ODR = PL::ADXL355_OutputDataRate::odr500;// 500 Hz (low-pass filter: -1.83dB at 125 Hz)
unsigned int dtRead = 1000 / 500; //delta T msec
double FIR[BufNum] = {
  #include "FIR500_cut50.csv"
};
//auto HPF = PL::ADXL355_HpfFrequency::none;// high-pass filter disabled
auto HPF = PL::ADXL355_HpfFrequency::hpf0_0954;// -3dB at ODR*0.0954e-4 = 500*0.0954e-4 = 0.00477
auto syncTime = PL::ADXL355_Synchronization::internal;// internal ///externalWithInterpolation; /// external with interpolation filter

//==============================================================================

// define two tasks
void TaskRead( void *pvParameters );
void TaskSave( void *pvParameters );
// queue
xQueueHandle xQueue;

//==============================================================================

void txtWrite(const char *string, uint16_t color) {
    M5.Lcd.fillScreen(color);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.setTextDatum(0);
    M5.Lcd.drawString(string, 0, 2, 4);
}
//==============================================================================

void createPath() {
  fileDate = dt.date.date; //Create a folder every day
  sprintf(filePath,"/%04d%02d%02d",
          dt.date.year,
          dt.date.month,
          fileDate);
  SD.mkdir(filePath);
}
//==============================================================================

void createFile() {
  fileDateTime = dt.time.minutes; //Create a file every minute
  sprintf(fileName, "%s/%02d%02d.csv",
          filePath,
          dt.time.hours,
          fileDateTime);
          //dt.time.seconds);
  f = SD.open(fileName, FILE_WRITE);
  if (!f) {
    Serial.print("ERROR: Can't open the file");
    while (1) ;    
  }
  f.println(accHeader);
}
//==============================================================================

void Set_RTC() {
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0,0);
  if (!M5.Rtc.isEnabled())
  {
    M5.Lcd.println("RTC not found.");
    delay(500);
  }
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0,0);
  M5.Lcd.println("RTC found.");
  delay(1000);
  
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0,0);
  M5.Lcd.print("WiFi: Searching.");
  WiFi.begin( WIFI_SSID, WIFI_PASSWORD );
  while (WiFi.status() != WL_CONNECTED)
  {
    M5.Lcd.print('.');
    delay(1000);
  }
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0,0);
  M5.Lcd.print("Wifi: Connected.");

  configTzTime(NTP_TIMEZONE, NTP_SERVER1, NTP_SERVER2, NTP_SERVER3);

  #if SNTP_ENABLED
    while (sntp_get_sync_status() != SNTP_SYNC_STATUS_COMPLETED)
    {
      delay(1000);
    }
  #else
    delay(1600);
    struct tm timeInfo;
    while (!getLocalTime(&timeInfo, 1000))
    {
    };
  #endif

  time_t t = time(nullptr)+1; // Advance one second.
  while (t > time(nullptr));  // Synchronization in seconds
  M5.Rtc.setDateTime( localtime( &t )); // for local timezone.

  // Show 10sec
  M5.Lcd.fillScreen(BLACK);
  for (int i = 100; i > 0; --i) {
    static constexpr const char* const wd[7] = {"Sun","Mon","Tue","Wed","Thr","Fri","Sat"};
    delay(100);
    auto dt = M5.Rtc.getDateTime();
    M5.Lcd.setCursor(0,0);
    M5.Lcd.printf("RTC   : %04d/%02d/%02d (%s) %02d: %02d: %02d"
                , dt.date.year
                , dt.date.month
                , dt.date.date
                , wd[dt.date.weekDay]
                , dt.time.hours
                , dt.time.minutes
                , dt.time.seconds
                );
    /// ESP32 internal timer
    auto t = time(nullptr);
    //auto tm = gmtime(&t);
    auto tm = localtime(&t); // for local timezone.
    M5.Lcd.setCursor(0,30);
    M5.Lcd.printf("ESP32: %04d/%02d/%02d (%s) %02d: %02d: %02d"
          , tm->tm_year+1900, tm->tm_mon+1, tm->tm_mday
          , wd[tm->tm_wday]
          , tm->tm_hour, tm->tm_min, tm->tm_sec
          );
    M5.Lcd.setCursor(0,70);
    M5.Lcd.print("The next program will start soon.");
  } 
}
//==============================================================================

void setup() {
  // Initialize M5
  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;
  cfg.clear_display = true;
  cfg.output_power = false;
  cfg.internal_imu = false;
  cfg.internal_rtc = true;
  cfg.internal_spk = false;
  cfg.external_imu = false;
  cfg.external_rtc = false;
  cfg.external_spk = false;
  cfg.led_brightness = 0;
  M5.begin(cfg);
  M5.Lcd.setBrightness(100);
  //M5.Lcd.sleep(); //<<<----------- don't show data on the screen

  //==============================================================================
  // Select whether to reset the RTC
  // This is your button on the screen. You can adjust position and size as needed.
  M5.Lcd.fillRoundRect (60, 120, 200, 50, 10, RED ); 
  M5.Lcd.fillRoundRect (64, 124, 192, 42, 10, ORANGE); 
  M5.Lcd.setCursor(95, 133);
  M5.Lcd.setTextFont(4);
  M5.Lcd.setTextColor(BLACK, BLACK); 
  M5.Lcd.println("Press Here!");
  //reset font
  M5.Lcd.setTextFont(2);
  M5.Lcd.setTextColor(WHITE, BLACK);
  // Show 30sec countdown
  for (float i = 3000.; i > 0.; --i) {
    M5.update();
    int16_t x = M5.Touch.getDetail().x;
    int16_t y = M5.Touch.getDetail().y;
      
    if (x >= 60 && x <= 260 && y >= 120 && y <= 170) {
      Set_RTC();
      break;
    }

    M5.Lcd.setCursor(0, 10);
    //M5.Lcd.setTextSize(2);
    M5.Lcd.println("Want to connect to an NTP server over Wi-Fi and reset the RTC?");
    M5.Lcd.setCursor(0, 60);
    M5.Lcd.printf("Time remaining: %.2f\n", i / 100.);
  }
  //==============================================================================

  M5.In_I2C.release();

  // Initialize ADXL355
  adxl355.begin();
  // Set range
  adxl355.setRange(range);
  // Set ODR
  adxl355.setOutputDataRate(ODR);
  // Set HPF
  adxl355.setHpfFrequency(HPF);
  // Enable ExternalClock()
  //adxl355.enableExternalClock();
  //adxl355.disableExternalClock();
  // Set Synchronization
  adxl355.setSynchronization(syncTime);
  // Enable ADXL355 measurement
  adxl355.enableMeasurement();

  // Strat SD
  while (!SD.begin(GPIO_NUM_4, SPI, 10000000)) { //SPI: 1kHz-10MHz
    Serial.println("ERROR: SD CARD.");
    delay(100);
  }

  // Strat RTC
  dt = M5.Rtc.getDateTime();
  txtWrite("RTC begginig...", TFT_BLACK);
  while((dt.date.year < 2024) || (dt.date.year > 2025)) {
    Serial.println(dt.date.year);
    delay(10);//msec wait
    dt = M5.Rtc.getDateTime();
  }

  // Queue size #size_of 
  xQueue = xQueueCreate(1, 16);
  // Set up two tasks to run independently.
  if (xQueue != NULL){
    xTaskCreatePinnedToCore(
      TaskRead
      ,  "ReadADXL355"   // Task name
      ,  8192 // Stack size
      ,  NULL
      ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,  NULL 
      //,  APP_CPU_NUM); //Core 1, without WDT
      ,  PRO_CPU_NUM);
    xTaskCreatePinnedToCore(
      TaskSave
      ,  "SaveData2SDCard"
      ,  8192 // Stack size
      ,  NULL
      ,  3  // Priority
      ,  NULL 
      //,  PRO_CPU_NUM); //Core 0	with WDT
      ,APP_CPU_NUM);
  }
  else {
    while(1){
      Serial.println("Failed to create queue.");
    }
  }
}
//==============================================================================

void loop() { //priolity 2, APP_CPU_NUM
  delay(1000);
}
//==============================================================================

void TaskRead(void *pvParameters) {
  auto accelerations = adxl355.getAccelerations();;
  unsigned long sTime = 1;
  unsigned int i = 1;
  unsigned int j = 1;
  unsigned int k = 0;
  unsigned int l = 0;
  String accData;
  
  // TaskRead start time
  dt = M5.Rtc.getDateTime();
  txtWrite("waite starting...", TFT_BLACK);
  while(dt.time.seconds!=0) {
    //Serial.println("waite starting...");
    delay(1);//msec wait
    dt = M5.Rtc.getDateTime();
  }

  portTickType xLastWakeTimeSend = xTaskGetTickCount();

  for (;;) {
    while (j <= dtWrite / dtRead){
      vTaskDelayUntil( &xLastWakeTimeSend, dtRead / portTICK_RATE_MS );
      sTime = millis();
      // Read high frequency accelerations
      accelerations = adxl355.getAccelerations();
      AccX = accelerations.x;
      AccY = accelerations.y;
      AccZ = accelerations.z;
      // Store accelerations
      if(BufIndex >=BufNum) BufIndex=0;
      RingBufX[BufIndex] = AccX;
      RingBufY[BufIndex] = AccY;
      RingBufZ[BufIndex] = AccZ;
      BufIndex++;
      j++;
    }
    j = 1;

    // FIR Filter & decimation
    l = BufIndex-1;
    for(k = 0; k < BufNum; k++){
      if(l<k) l = BufNum-1+k;
      AccFirX += FIR[k] * RingBufX[l-k];
      AccFirY += FIR[k] * RingBufY[l-k];
      AccFirZ += FIR[k] * RingBufZ[l-k];
    }
    accData += "\n";
    accData += String(sTime);
	  accData += String(",");
	  accData += String(AccFirX);
	  accData += String(",");
	  accData += String(AccFirY);
	  accData += String(",");
	  accData += String(AccFirZ);
    /**/
    Serial.print(sTime);
	  Serial.print(", ");
	  Serial.print(AccFirX);
	  Serial.print(", ");
	  Serial.print(AccFirY);
	  Serial.print(", ");
	  Serial.println(AccFirZ);
    /**/
    AccFirX = 0;
    AccFirY = 0;
    AccFirZ = 0;

    if ( i >= hz * SDWriteTime){
      //Serial.println(sizeof(accData)); //Check Queue size
      xQueueSendToBack(xQueue, &accData, 0);//1 / portTICK_RATE_MS);
      accData = "";
      i = 1;
    }
    else{
      i++;
    }
    //vTaskDelayUntil( &xLastWakeTimeSend, 1000 / hz / portTICK_RATE_MS );
  }
}
//==============================================================================

void TaskSave(void *pvParameters) {
  String recData;
  const char* cstr;

  // TaskSave start time
    delay(5);//msec wait

  for (;;){
    xQueueReceive(xQueue, &recData, SDWriteTime * 1000 / portTICK_RATE_MS);
    //Serial.print(recData);
    recData += "\n";
    f.print(recData.substring(1));
	  f.close();
    cstr = recData.c_str(); //<<<---------------- show data on the screen
    txtWrite(cstr, BLACK); //<<<-------------
    delay(SDWriteTime*1000/10); //<<<----------
    M5.Lcd.fillScreen(BLACK); //<<<---------- clear screen

    recData="";
    dt = M5.Rtc.getDateTime();
    M5.Lcd.setCursor(0,0); //<<<----------------- show time on the screen
    M5.Lcd.setTextFont(4); //<<<-----------
    M5.Lcd.printf("%04d/%02d/%02d  %02d:%02d:%02d"  //<<<-----------
               , dt.date.year
               , dt.date.month
               , dt.date.date
               , dt.time.hours
               , dt.time.minutes
               , dt.time.seconds
               );

    if (dt.time.minutes != fileDateTime){
      if (dt.date.date != fileDate){ // initialy fileDate = 0
	      createPath();
	    }
      createFile();
    }
    else{ 
      f = SD.open(fileName, FILE_APPEND);
    }
    delay(SDWriteTime*1000/10); //<<<-----------
    M5.Lcd.fillScreen(BLACK); //<<<----------- clear screen
  }
}