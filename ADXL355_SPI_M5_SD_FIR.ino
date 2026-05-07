#include <SD.h>
#include <M5Unified.h>
#include <M5_ADXL355.h>

// Self-contained minimal FTP server for the Data Dump mode.
// Uses only WiFi.h (WiFiServer/WiFiClient) - no external library.

//==============================================================================
// for RTC
// M5Unified Sample
// https://github.com/m5stack/M5Unified/blob/master/examples/Basic/Rtc/Rtc.ino

#if defined ( ARDUINO )
#define NTP_TIMEZONE  "JST-9"
#define NTP_SERVER1   "ntp2.jst.mfeed.ad.jp"
#define NTP_SERVER2   "ntp1.jst.mfeed.ad.jp"
#define NTP_SERVER3   "ntp.nict.jp"

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

// Output sampling rate (compile-time): 100 or 500 Hz.
//   100 Hz : ADXL355 ODR=500 Hz, 201-tap FIR, 50 Hz cutoff (FIR500_cut50.csv).
//   500 Hz : ADXL355 ODR=1000 Hz, 80-tap minimum-phase FIR (Telemetra min500.cf,
//            passband 200 Hz, stopband 250 Hz @ -120 dB).
#define SAMPLE_HZ 100

unsigned int hz = SAMPLE_HZ;
unsigned int dtWrite = 1000 / hz;
unsigned int SDWriteTime = 15;

// dt is shared between TaskRead and TaskSave; protect with portMUX.
auto dt = M5.Rtc.getDateTime();
portMUX_TYPE dtMux = portMUX_INITIALIZER_UNLOCKED;

char filePath[10];
char fileName[30];
int fileDate = 0;
int fileDateTime;
File f;

const String accHeader = "Time(msec),x(cm/s2),y(cm/s2),z(cm/s2)";
double AccX = 0.;
double AccY = 0.;
double AccZ = 0.;
double AccFirX = 0.;
double AccFirY = 0.;
double AccFirZ = 0.;

// FIR taps and coefficients depend on SAMPLE_HZ.
// Files live in the sketch's FIR/ subfolder.
#if SAMPLE_HZ == 100
  #define FIR_TAPS 201
  static const double FIR_COEF[FIR_TAPS] = {
    #include "FIR/FIR500_cut50.csv"
  };
#elif SAMPLE_HZ == 500
  #include "FIR/min500.cf"   // declares: double min500[80] = {...};
  #define FIR_TAPS 80
  #define FIR_COEF min500
#else
  #error "SAMPLE_HZ must be 100 or 500"
#endif

double RingBufX[FIR_TAPS] = {};
double RingBufY[FIR_TAPS] = {};
double RingBufZ[FIR_TAPS] = {};
unsigned int BufIndex = 0;

// Initial capacity for accData (~30 bytes/line * hz * SDWriteTime + margin).
const size_t ACCDATA_RESERVE = (size_t)SAMPLE_HZ * 15 * 32;

// Drop count (missed batches when the SD queue is full) .
volatile uint32_t dropCount = 0;
volatile uint32_t batchOkCount = 0;

// Latest FIR output for LCD display (vibration check).
double latestX = 0.0;
double latestY = 0.0;
double latestZ = 0.0;

//==============================================================================

// Create an instance of ADXL355
//PL::ADXL355(SCK, MISO, MOSI, SS)
PL::ADXL355 adxl355(26, 36, 32, 33);
auto range = PL::ADXL355_Range::range2g;// ADXL355 range: +/- 2 g

#if SAMPLE_HZ == 100
auto ODR = PL::ADXL355_OutputDataRate::odr500;   // sensor LPF -3 dB @ 125 Hz
unsigned int dtRead = 1000 / 500;                // 2 ms
#elif SAMPLE_HZ == 500
auto ODR = PL::ADXL355_OutputDataRate::odr1000;  // sensor LPF -3 dB @ 250 Hz
unsigned int dtRead = 1000 / 1000;               // 1 ms
#endif

auto HPF = PL::ADXL355_HpfFrequency::hpf0_0954;
auto syncTime = PL::ADXL355_Synchronization::internal;

//==============================================================================

// define two tasks
void TaskRead( void *pvParameters );
void TaskSave( void *pvParameters );
// queue
xQueueHandle xQueue;

// FTP server state (Data Dump mode). See MiniFTP definitions below.
WiFiServer ftpCtrlSrv(21);
WiFiServer ftpDataSrv(50000);
static const uint16_t FTP_PASV_PORT = 50000;

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

// Safe TF-open helper that retries on failure.
// Retries every 1 second, remounts the TF every 30 seconds
//  to recover from  transient contact issues or write stalls.
static File openSDFileSafe(const char *path, const char *mode) {
  File f;
  uint32_t retry = 0;
  while (true) {
    f = SD.open(path, mode);
    if (f) {
      if (retry > 0) {
        Serial.printf("SD recovered after %u retries\n", retry);
      }
      return f;
    }
    retry++;
    Serial.printf("SD open '%s' failed, retry %u\n", path, retry);
    M5.Lcd.fillRect(0, 200, 320, 40, RED);
    M5.Lcd.setTextFont(2);
    M5.Lcd.setTextColor(WHITE, RED);
    M5.Lcd.setCursor(0, 200);
    M5.Lcd.printf("SD ERROR retry=%u", (unsigned)retry);
    M5.Lcd.setCursor(0, 220);
    M5.Lcd.print("Check SD card!");
    delay(1000);
    if (retry % 30 == 0) {
      Serial.println("Remounting SD...");
      SD.end();
      delay(500);
      SD.begin(GPIO_NUM_4, SPI, 10000000);
    }
  }
}
//==============================================================================

void createFile() {
  fileDateTime = dt.time.minutes; //Create a file every minute
  sprintf(fileName, "%s/%02d%02d.csv",
          filePath,
          dt.time.hours,
          fileDateTime);
  f = openSDFileSafe(fileName, FILE_WRITE);
  f.println(accHeader);
}
//==============================================================================

// Post-set confirmation screen, shown for 10 seconds.
static void rtcConfirmScreen() {
  static constexpr const char* const wd[7] = {"Sun","Mon","Tue","Wed","Thr","Fri","Sat"};

  M5.Lcd.fillScreen(WHITE);
  M5.Lcd.setTextColor(BLACK, WHITE);
  M5.Lcd.setTextFont(2);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.print("RTC time:");
  M5.Lcd.setCursor(0, 90);
  M5.Lcd.print("ESP32 time:");

  int prevSec = -1;
  for (int i = 100; i > 0; --i) {
    delay(100);
    auto rtcDt = M5.Rtc.getDateTime();
    auto sysT  = time(nullptr);
    auto sysTm = localtime(&sysT);

    M5.Lcd.setTextFont(4);
    M5.Lcd.setTextColor(BLACK, WHITE);
    M5.Lcd.setCursor(0, 18);
    M5.Lcd.printf("%04d/%02d/%02d (%s)",
                  rtcDt.date.year, rtcDt.date.month, rtcDt.date.date, wd[rtcDt.date.weekDay]);
    M5.Lcd.setCursor(0, 50);
    M5.Lcd.printf("%02d:%02d:%02d",
                  rtcDt.time.hours, rtcDt.time.minutes, rtcDt.time.seconds);

    M5.Lcd.setCursor(0, 108);
    M5.Lcd.printf("%04d/%02d/%02d (%s)",
                  sysTm->tm_year+1900, sysTm->tm_mon+1, sysTm->tm_mday, wd[sysTm->tm_wday]);
    M5.Lcd.setCursor(0, 140);
    M5.Lcd.printf("%02d:%02d:%02d",
                  sysTm->tm_hour, sysTm->tm_min, sysTm->tm_sec);

    int sec = i / 10;
    if (sec != prevSec) {
      M5.Lcd.setTextFont(2);
      M5.Lcd.setCursor(0, 200);
      M5.Lcd.printf("Measurement starts in %2d sec", sec);
      prevSec = sec;
    }
  }
}

//==============================================================================
void Set_RTC() {
  M5.Lcd.fillScreen(WHITE);
  M5.Lcd.setCursor(0,0);
  M5.Lcd.setTextColor(BLACK, WHITE);
  M5.Lcd.setTextFont(2);
  if (!M5.Rtc.isEnabled())
  {
    M5.Lcd.println("RTC not found.");
    delay(500);
  }
  M5.Lcd.fillScreen(WHITE);
  M5.Lcd.setCursor(0,0);
  M5.Lcd.println("RTC found.");
  delay(1000);

  M5.Lcd.setCursor(0,0);
  M5.Lcd.print("WiFi: Searching.");
  WiFi.begin();
  while (WiFi.status() != WL_CONNECTED) {
    M5.Lcd.print('.');
    delay(1000);
  }
  M5.Lcd.setCursor(0,0);
  M5.Lcd.print("Wifi: Connected.");

  configTzTime(NTP_TIMEZONE, NTP_SERVER1, NTP_SERVER2, NTP_SERVER3);

  #if SNTP_ENABLED
    while (sntp_get_sync_status() != SNTP_SYNC_STATUS_COMPLETED) {
      delay(1000);
    }
  #else
    delay(1600);
    struct tm timeInfo;
    while (!getLocalTime(&timeInfo, 1000)) {
    };
  #endif

  time_t t = time(nullptr) + 1;
  while (t > time(nullptr));
  M5.Rtc.setDateTime(localtime(&t));

  rtcConfirmScreen();
  WiFi.disconnect(true);
}
//==============================================================================
void Set_WiFi(){
  M5.Lcd.fillScreen(WHITE);
  M5.Lcd.setTextColor(BLACK, WHITE);
  M5.Lcd.setTextFont(4);

  M5.Lcd.setCursor(0, 5);
  M5.Lcd.println("Use your phone");
  M5.Lcd.setCursor(0, 35);
  M5.Lcd.println("to select Wi-Fi.");

  WiFi.mode(WIFI_AP_STA);
  WiFi.beginSmartConfig();

  M5.Lcd.setCursor(0, 90);
  M5.Lcd.println("Waiting for");
  M5.Lcd.setCursor(0, 120);
  M5.Lcd.println("SmartConfig...");

  while (!WiFi.smartConfigDone()) {
    delay(500);
  }
  Set_RTC();
}
//==============================================================================

// Manual RTC setting UI.
void Manual_Set() {
  int year   = 2026;
  int month  = 1;
  int day    = 1;
  int hour   = 0;
  int minute = 0;
  int second = 0;

  // Button layout (320x240)
  const int xs[6]   = {   5,  80, 132, 184, 236, 288 };
  const int colW[6] = {  70,  48,  48,  48,  48,  30 };
  const int upY  = 30;
  const int dnY  = 120;
  const int btnH = 40;

  auto drawAll = [&](){
    M5.Lcd.fillScreen(WHITE);
    M5.Lcd.setTextColor(BLACK, WHITE);
    M5.Lcd.setTextFont(2);

    for (int i = 0; i < 6; i++) {
      M5.Lcd.fillRoundRect(xs[i], upY, colW[i], btnH, 5, ORANGE);
      M5.Lcd.setCursor(xs[i] + colW[i]/2 - 5, upY + btnH/2 - 8);
      M5.Lcd.setTextColor(BLACK, ORANGE);
      M5.Lcd.print("+");
    }
    for (int i = 0; i < 6; i++) {
      M5.Lcd.fillRoundRect(xs[i], dnY, colW[i], btnH, 5, ORANGE);
      M5.Lcd.setCursor(xs[i] + colW[i]/2 - 5, dnY + btnH/2 - 8);
      M5.Lcd.setTextColor(BLACK, ORANGE);
      M5.Lcd.print("-");
    }
    M5.Lcd.setTextColor(BLACK, WHITE);
    const char *hdr[6] = {"Year", "Mon", "Day", "Hour", "Min", "Sec"};
    for (int i = 0; i < 6; i++) {
      M5.Lcd.setCursor(xs[i] + 4, 5);
      M5.Lcd.print(hdr[i]);
    }
    M5.Lcd.setTextFont(4);
    char buf[8];
    int vals[6] = {year, month, day, hour, minute, second};
    for (int i = 0; i < 6; i++) {
      M5.Lcd.fillRect(xs[i], 75, colW[i], 40, WHITE);
      M5.Lcd.setCursor(xs[i] + 2, 80);
      if (i == 0) sprintf(buf, "%04d", vals[i]);
      else        sprintf(buf, "%02d", vals[i]);
      M5.Lcd.print(buf);
    }
    // SET / CANCEL
    M5.Lcd.fillRoundRect(20,  195, 130, 40, 8, GREEN);
    M5.Lcd.fillRoundRect(170, 195, 130, 40, 8, RED);
    M5.Lcd.setTextFont(4);
    M5.Lcd.setTextColor(BLACK, GREEN);
    M5.Lcd.setCursor(60, 200);
    M5.Lcd.print("SET");
    M5.Lcd.setTextColor(WHITE, RED);
    M5.Lcd.setCursor(190, 200);
    M5.Lcd.print("CANCEL");
    M5.Lcd.setTextColor(BLACK, WHITE);
  };

  auto adjust = [&](int idx, int delta){
    switch (idx) {
      case 0: year   = constrain(year + delta, 2024, 2099); break;
      case 1: month  = ((month - 1 + delta + 12) % 12) + 1; break;
      case 2: day    = ((day   - 1 + delta + 31) % 31) + 1; break;
      case 3: hour   = (hour   + delta + 24) % 24; break;
      case 4: minute = (minute + delta + 60) % 60; break;
      case 5: second = (second + delta + 60) % 60; break;
    }
  };

  drawAll();

  while (true) {
    M5.update();
    auto t = M5.Touch.getDetail();
    if (t.wasPressed()) {
      int xt = t.x;
      int yt = t.y;

      for (int i = 0; i < 6; i++) {
        if (xt >= xs[i] && xt <= xs[i] + colW[i] &&
            yt >= upY  && yt <= upY  + btnH) {
          adjust(i, +1);
          drawAll();
        }
      }
      for (int i = 0; i < 6; i++) {
        if (xt >= xs[i] && xt <= xs[i] + colW[i] &&
            yt >= dnY  && yt <= dnY  + btnH) {
          adjust(i, -1);
          drawAll();
        }
      }
      if (xt >= 20 && xt <= 150 && yt >= 195 && yt <= 235) {
        auto newdt = M5.Rtc.getDateTime();
        newdt.date.year   = year;
        newdt.date.month  = month;
        newdt.date.date   = day;
        newdt.time.hours  = hour;
        newdt.time.minutes= minute;
        newdt.time.seconds= second;
        M5.Rtc.setDateTime(&newdt);

        struct tm tmSet = {};
        tmSet.tm_year = year - 1900;
        tmSet.tm_mon  = month - 1;
        tmSet.tm_mday = day;
        tmSet.tm_hour = hour;
        tmSet.tm_min  = minute;
        tmSet.tm_sec  = second;
        time_t tt = mktime(&tmSet);
        struct timeval tv = { tt, 0 };
        settimeofday(&tv, nullptr);

        // Same 10-second confirmation screen as Set_RTC.
        rtcConfirmScreen();
        return;
      }
      if (xt >= 170 && xt <= 300 && yt >= 195 && yt <= 235) {
        return;
      }
    }
    delay(20);
  }
}

//==============================================================================

// MiniFTP state (control connection, current cwd, auth flag, login arg).
static WiFiClient ftpCli;
static String    ftpCwd     = "/";
static bool      ftpAuthed  = false;
static String    ftpUserArg;
static IPAddress ftpLocalIP;

static const char *FTP_USER = "m5";
static const char *FTP_PASS = "m5";

// Resolve an FTP path argument relative to ftpCwd.
static String ftpResolve(const String &arg) {
  if (arg.length() == 0) return ftpCwd;
  if (arg.startsWith("/")) return arg;
  if (ftpCwd == "/") return "/" + arg;
  return ftpCwd + "/" + arg;
}

// Send a code+message line on the FTP control connection.
static void ftpReply(int code, const char *msg) {
  ftpCli.printf("%d %s\r\n", code, msg);
}

// Wait for a passive-mode data client to connect (timeout in ms).
static WiFiClient ftpAcceptData(uint32_t timeoutMs = 5000) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    WiFiClient c = ftpDataSrv.accept();
    if (c) return c;
    delay(10);
  }
  return WiFiClient();
}
//==============================================================================

// Handle a single command line received on the FTP control connection.
static void ftpHandleCmd(const String &line) {
  int sp = line.indexOf(' ');
  String cmd = (sp == -1) ? line : line.substring(0, sp);
  String arg = (sp == -1) ? ""   : line.substring(sp + 1);
  cmd.toUpperCase();
  arg.trim();
  Serial.printf("FTP> %s %s\n", cmd.c_str(), arg.c_str());

  if (cmd == "USER") {
    ftpUserArg = arg;
    ftpReply(331, "Need password");
  } else if (cmd == "PASS") {
    if (ftpUserArg == FTP_USER && arg == FTP_PASS) {
      ftpAuthed = true;
      ftpReply(230, "Logged in");
    } else {
      ftpReply(530, "Login incorrect");
    }
  } else if (!ftpAuthed) {
    ftpReply(530, "Not logged in");
  } else if (cmd == "SYST") {
    ftpReply(215, "UNIX Type: L8");
  } else if (cmd == "FEAT") {
    ftpCli.println("211-Features:");
    ftpCli.println(" PASV");
    ftpCli.println(" SIZE");
    ftpCli.println(" UTF8");
    ftpCli.println("211 End");
  } else if (cmd == "OPTS") {
    ftpReply(200, "OK");
  } else if (cmd == "TYPE") {
    ftpReply(200, "Type set");
  } else if (cmd == "PWD" || cmd == "XPWD") {
    ftpCli.printf("257 \"%s\" is current directory\r\n", ftpCwd.c_str());
  } else if (cmd == "CWD") {
    String np = ftpResolve(arg);
    File f = SD.open(np);
    if (f && f.isDirectory()) {
      ftpCwd = np;
      ftpReply(250, "Directory changed");
    } else {
      ftpReply(550, "Directory not found");
    }
    if (f) f.close();
  } else if (cmd == "CDUP") {
    int sl = ftpCwd.lastIndexOf('/');
    if (sl > 0) ftpCwd = ftpCwd.substring(0, sl);
    else        ftpCwd = "/";
    ftpReply(200, "Up");
  } else if (cmd == "PASV") {
    // Make sure the data server is listening on the fixed PASV port.
    ftpDataSrv.begin();
    uint8_t pH = (FTP_PASV_PORT >> 8) & 0xff;
    uint8_t pL = FTP_PASV_PORT & 0xff;
    ftpCli.printf("227 Entering Passive Mode (%u,%u,%u,%u,%u,%u)\r\n",
                  ftpLocalIP[0], ftpLocalIP[1], ftpLocalIP[2], ftpLocalIP[3],
                  pH, pL);
  } else if (cmd == "LIST" || cmd == "NLST") {
    ftpReply(150, "Opening data connection");
    WiFiClient dc = ftpAcceptData();
    if (!dc) { ftpReply(425, "Can't open data connection"); return; }
    bool namesOnly = (cmd == "NLST");
    File dir = SD.open(ftpCwd);
    if (dir && dir.isDirectory()) {
      while (true) {
        File entry = dir.openNextFile();
        if (!entry) break;
        String name = entry.name();
        int sl = name.lastIndexOf('/');
        if (sl >= 0) name = name.substring(sl + 1);
        if (namesOnly) {
          dc.printf("%s\r\n", name.c_str());
        } else {
          // UNIX-style ls -l output (Explorer parses this format).
          const char *mode = entry.isDirectory() ? "drwxr-xr-x" : "-rw-r--r--";
          dc.printf("%s 1 m5 m5 %u Jan 01  2026 %s\r\n",
                    mode, (unsigned)entry.size(), name.c_str());
        }
        entry.close();
      }
      dir.close();
    }
    dc.stop();
    ftpReply(226, "Transfer complete");
  } else if (cmd == "RETR") {
    String path = ftpResolve(arg);
    File f = SD.open(path);
    if (!f || f.isDirectory()) {
      ftpReply(550, "File not found");
      if (f) f.close();
      return;
    }
    ftpReply(150, "Opening data connection");
    WiFiClient dc = ftpAcceptData();
    if (!dc) { ftpReply(425, "Can't open data connection"); f.close(); return; }
    uint8_t buf[1460];
    while (f.available()) {
      int n = f.read(buf, sizeof(buf));
      if (n <= 0) break;
      dc.write(buf, n);
    }
    f.close();
    dc.stop();
    ftpReply(226, "Transfer complete");
  } else if (cmd == "SIZE") {
    String path = ftpResolve(arg);
    File f = SD.open(path);
    if (f && !f.isDirectory()) {
      ftpCli.printf("213 %u\r\n", (unsigned)f.size());
    } else {
      ftpReply(550, "Not found");
    }
    if (f) f.close();
  } else if (cmd == "NOOP") {
    ftpReply(200, "OK");
  } else if (cmd == "QUIT") {
    ftpReply(221, "Bye");
    ftpCli.stop();
  } else {
    ftpReply(502, "Not implemented");
  }
}
//==============================================================================

// Process pending FTP control activity. Call repeatedly from the main loop.
static void ftpHandle() {
  if (!ftpCli || !ftpCli.connected()) {
    WiFiClient incoming = ftpCtrlSrv.accept();
    if (incoming) {
      ftpCli = incoming;
      ftpAuthed = false;
      ftpCwd    = "/";
      ftpReply(220, "M5 FTP Ready");
    }
    return;
  }
  if (ftpCli.available()) {
    String line = ftpCli.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      ftpHandleCmd(line);
    }
  }
}
//==============================================================================

// Data Dump mode: SoftAP + minimal FTP server.
// Connect Wi-Fi to "M5-SEISMO" (m5seismo) and open
// ftp://192.168.4.1 in Windows Explorer (user m5 / pass m5).
void Data_Dump_FTP() {
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextFont(2);
  M5.Lcd.println("Data Dump (AP+FTP)");
  M5.Lcd.println();

  M5.Lcd.print("Init SD...");
  uint32_t t0 = millis();
  while (!SD.begin(GPIO_NUM_4, SPI, 10000000)) {
    if (millis() - t0 > 5000) {
      M5.Lcd.fillScreen(RED);
      M5.Lcd.setCursor(0, 0);
      M5.Lcd.setTextColor(WHITE, RED);
      M5.Lcd.println("SD INIT FAILED.");
      M5.Lcd.println("Reset to retry.");
      while (true) delay(1000);
    }
    delay(100);
  }
  M5.Lcd.println(" OK");

  WiFi.mode(WIFI_AP);
  IPAddress apIP(192, 168, 4, 1);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP("M5-SEISMO", "m5seismo");

  ftpLocalIP = apIP;
  ftpCtrlSrv.begin();
  ftpDataSrv.begin();
  ftpAuthed = false;
  ftpCwd    = "/";

  M5.Lcd.printf("SSID: M5-SEISMO\n");
  M5.Lcd.printf("PASS: m5seismo\n");
  M5.Lcd.println();
  M5.Lcd.printf("URL : ftp://192.168.4.1\n");
  M5.Lcd.printf("User: m5 / Pass: m5\n");
  M5.Lcd.println();
  M5.Lcd.println("Open in Explorer:");
  M5.Lcd.println("ftp://m5:m5@192.168.4.1");

  const int rbX = 60, rbY = 190, rbW = 200, rbH = 40;
  M5.Lcd.fillRoundRect(rbX,     rbY,     rbW,     rbH,     8, RED);
  M5.Lcd.fillRoundRect(rbX + 4, rbY + 4, rbW - 8, rbH - 8, 8, ORANGE);
  M5.Lcd.setTextFont(4);
  M5.Lcd.setTextColor(BLACK, ORANGE);
  M5.Lcd.setCursor(rbX + 70, rbY + 8);
  M5.Lcd.print("Reset");

  while (true) {
    ftpHandle();
    M5.update();
    auto td = M5.Touch.getDetail();
    if (td.wasPressed()) {
      if (td.x >= rbX && td.x <= rbX + rbW &&
          td.y >= rbY && td.y <= rbY + rbH) {
        M5.Lcd.fillScreen(BLACK);
        M5.Lcd.setTextFont(4);
        M5.Lcd.setTextColor(WHITE, BLACK);
        M5.Lcd.setCursor(0, 100);
        M5.Lcd.println("Restarting...");
        delay(500);
        ESP.restart();
      }
    }
    delay(1);
  }
}

//==============================================================================

void setup() {
  // Initialize M5
  auto cfg = M5.config();
  cfg.serial_baudrate = 921600;
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

  //==============================================================================
  M5.Lcd.fillScreen(WHITE);
  const int btnX = 60;
  const int btnW = 200;
  const int btnH = 40;
  const int btnGap = 5;
  const int btnR = 8;
  const int btnY0 = 40;

  struct Btn { const char *label; int y; };
  Btn btns[4] = {
    {"Wi-Fi Setting", btnY0 + 0 * (btnH + btnGap)},
    {"Reset RTC",     btnY0 + 1 * (btnH + btnGap)},
    {"Manual Set",    btnY0 + 2 * (btnH + btnGap)},
    {"Data Dump",     btnY0 + 3 * (btnH + btnGap)},
  };
  for (int i = 0; i < 4; i++) {
    M5.Lcd.fillRoundRect(btnX,     btns[i].y,     btnW,     btnH,     btnR, RED);
    M5.Lcd.fillRoundRect(btnX + 4, btns[i].y + 4, btnW - 8, btnH - 8, btnR, ORANGE);
    M5.Lcd.setTextFont(4);
    M5.Lcd.setTextColor(BLACK, ORANGE);
    M5.Lcd.setCursor(btnX + 25, btns[i].y + 8);
    M5.Lcd.print(btns[i].label);
  }
  M5.Lcd.setTextColor(BLACK, WHITE);

  M5.Lcd.setTextFont(4);
  M5.Lcd.setTextColor(BLACK, WHITE);
  M5.Lcd.setCursor(0, 5);
  M5.Lcd.print("Tap a button:");

  int prevSec = -1;
  for (int i = 300; i > 0; --i) {
    M5.update();
    auto td = M5.Touch.getDetail();
    int xt = td.x, yt = td.y;
    bool dispatched = false;
    if (td.wasPressed()) {
      for (int b = 0; b < 4; b++) {
        if (xt >= btnX && xt <= btnX + btnW &&
            yt >= btns[b].y && yt <= btns[b].y + btnH) {
          switch (b) {
            case 0: Set_WiFi();      break;
            case 1: Set_RTC();       break;
            case 2: Manual_Set();    break;
            case 3: Data_Dump_FTP(); break;
          }
          dispatched = true;
          break;
        }
      }
    }
    if (dispatched) break;

    int sec = i / 10;
    if (sec != prevSec) {
      M5.Lcd.setTextFont(4);
      M5.Lcd.setTextColor(BLACK, WHITE);
      M5.Lcd.setCursor(0, 215);
      M5.Lcd.printf("Wait... %2d sec", sec);
      prevSec = sec;
    }
    delay(100);
  }
  //==============================================================================

  // Initialize ADXL355
  adxl355.begin();
  adxl355.setRange(range);
  adxl355.setOutputDataRate(ODR);
  adxl355.setHpfFrequency(HPF);
  adxl355.setSynchronization(syncTime);
  adxl355.enableMeasurement();

  // Verify FIR coefficient sum at startup.
  double firSum = 0.;
  for (int k = 0; k < FIR_TAPS; k++) firSum += FIR_COEF[k];
  Serial.printf("FIR coef sum = %.6f\n", firSum);
  if (fabs(firSum - 1.0) > 0.01) {
    M5.Lcd.fillScreen(RED);
    M5.Lcd.setTextColor(WHITE, RED);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.setTextFont(2);
    M5.Lcd.printf("WARN: FIR sum = %.4f\n", firSum);
    M5.Lcd.println("DC gain mismatch.");
    delay(2000);
  }

  // Start SD
  while (!SD.begin(GPIO_NUM_4, SPI, 10000000)) {
    txtWrite("ERROR: SD CARD", BLACK);
    delay(100);
  }

  dt = M5.Rtc.getDateTime();
  txtWrite("Start RTC. waiting...", BLACK);
  while ((dt.date.year < 2026) || (dt.date.year > 2099)) {
    Serial.println(dt.date.year);
    delay(10);
    dt = M5.Rtc.getDateTime();
  }

  // Queue holds up to 3 String pointers to absorb transient SD delays.
  xQueue = xQueueCreate(3, sizeof(String*));
  if (xQueue != NULL){
    xTaskCreatePinnedToCore(
      TaskRead
      ,  "ReadADXL355"
      ,  8192
      ,  NULL
      ,  3
      ,  NULL
      ,  PRO_CPU_NUM);
    xTaskCreatePinnedToCore(
      TaskSave
      ,  "SaveData2SDCard"
      ,  8192
      ,  NULL
      ,  3
      ,  NULL
      ,  APP_CPU_NUM);
  }
  else {
    while(1){
      Serial.println("Failed to create queue.");
    }
  }
}
//==============================================================================

void loop() {
  delay(1000);
}
//==============================================================================

void TaskRead(void *pvParameters) {
  auto accelerations = adxl355.getAccelerations();
  unsigned long sTime = 1;
  unsigned int i = 1;
  unsigned int j = 1;
  unsigned int k = 0;
  unsigned int l = 0;
  String accData;
  accData.reserve(ACCDATA_RESERVE);

  auto localDt = M5.Rtc.getDateTime();
  txtWrite("waiting start...", BLACK);
  while (localDt.time.seconds != 0) {
    delay(1);
    localDt = M5.Rtc.getDateTime();
  }
  portENTER_CRITICAL(&dtMux);
  dt = localDt;
  portEXIT_CRITICAL(&dtMux);

  TickType_t xLastWakeTimeSend = xTaskGetTickCount();

  for (;;) {
    while (j <= dtWrite / dtRead) {
      vTaskDelayUntil(&xLastWakeTimeSend, dtRead / portTICK_PERIOD_MS);
      sTime = millis();
      accelerations = adxl355.getAccelerations();
      AccX = accelerations.x;
      AccY = accelerations.y;
      AccZ = accelerations.z;
      if (BufIndex >= FIR_TAPS) BufIndex = 0;
      RingBufX[BufIndex] = AccX;
      RingBufY[BufIndex] = AccY;
      RingBufZ[BufIndex] = AccZ;
      BufIndex++;
      j++;
    }
    j = 1;

    // FIR Filter & decimation
    l = BufIndex - 1;
    for (k = 0; k < (unsigned int)FIR_TAPS; k++) {
      if (l < k) l = FIR_TAPS - 1 + k;
      AccFirX += FIR_COEF[k] * RingBufX[l - k];
      AccFirY += FIR_COEF[k] * RingBufY[l - k];
      AccFirZ += FIR_COEF[k] * RingBufZ[l - k];
    }

    accData += '\n';
    accData += sTime;
    accData += ',';
    accData += AccFirX;
    accData += ',';
    accData += AccFirY;
    accData += ',';
    accData += AccFirZ;

    Serial.print(sTime);
    Serial.print(", ");
    Serial.print(AccFirX);
    Serial.print(", ");
    Serial.print(AccFirY);
    Serial.print(", ");
    Serial.println(AccFirZ);

    latestX = AccFirX;
    latestY = AccFirY;
    latestZ = AccFirZ;

    AccFirX = 0;
    AccFirY = 0;
    AccFirZ = 0;

    if (i >= hz * SDWriteTime) {
      String *snapshot = new String(accData);
      if (xQueueSendToBack(xQueue, &snapshot, 0) != pdTRUE) {
        delete snapshot;
        dropCount++;
      }
      accData = "";
      i = 1;
    }
    else {
      i++;
    }
  }
}
//==============================================================================

void TaskSave(void *pvParameters) {
  String *recData = nullptr;
  bool firstBatch = true;

  delay(5);

  for (;;) {
    // Check xQueueReceive return; skip write on timeout.
    if (xQueueReceive(xQueue, &recData, SDWriteTime * 1000 / portTICK_PERIOD_MS) != pdTRUE) {
      auto localDt = M5.Rtc.getDateTime();
      portENTER_CRITICAL(&dtMux);
      dt = localDt;      portEXIT_CRITICAL(&dtMux);
      continue;
    }

    // First batch: create the initial file using the post sec=0 RTC,
    // so the file name reflects the actual data start minute.
    if (firstBatch) {
      auto firstDt = M5.Rtc.getDateTime();
      portENTER_CRITICAL(&dtMux);
      dt = firstDt;
      portEXIT_CRITICAL(&dtMux);
      createPath();
      createFile();
      firstBatch = false;
    }

    *recData += '\n';
    f.print(recData->substring(1));
    f.close();
    batchOkCount++;

    auto localDt = M5.Rtc.getDateTime();
    portENTER_CRITICAL(&dtMux);
    dt = localDt;      portEXIT_CRITICAL(&dtMux);

    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.setTextFont(4);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.printf("%04d/%02d/%02d %02d:%02d:%02d",
                  localDt.date.year, localDt.date.month, localDt.date.date,
                  localDt.time.hours, localDt.time.minutes, localDt.time.seconds);

    M5.Lcd.setTextFont(2);
    M5.Lcd.setCursor(0, 40);
    M5.Lcd.printf("X: %8.2f cm/s2", latestX);
    M5.Lcd.setCursor(0, 60);
    M5.Lcd.printf("Y: %8.2f cm/s2", latestY);
    M5.Lcd.setCursor(0, 80);
    M5.Lcd.printf("Z: %8.2f cm/s2", latestZ);

    M5.Lcd.setCursor(0, 110);
    M5.Lcd.printf("OK  : %lu", (unsigned long)batchOkCount);
    // DROP = number of dropped batches (queue full from slow SD writes).
    M5.Lcd.setCursor(0, 130);
    M5.Lcd.setTextColor(dropCount ? RED : WHITE, BLACK);
    M5.Lcd.printf("DROP: %lu", (unsigned long)dropCount);
    M5.Lcd.setTextColor(WHITE, BLACK);

    delay(SDWriteTime * 1000 / 5);
    M5.Lcd.fillScreen(BLACK);

    delete recData;
    recData = nullptr;

    if (localDt.time.minutes != fileDateTime) {
      if (localDt.date.date != fileDate) {
        portENTER_CRITICAL(&dtMux);
        dt = localDt;
        portEXIT_CRITICAL(&dtMux);
        createPath();
      }
      portENTER_CRITICAL(&dtMux);
      dt = localDt;
      portEXIT_CRITICAL(&dtMux);
      createFile();
    }
    else {
      // Use openSDFileSafe here.
      f = openSDFileSafe(fileName, FILE_APPEND);
    }
  }
}
