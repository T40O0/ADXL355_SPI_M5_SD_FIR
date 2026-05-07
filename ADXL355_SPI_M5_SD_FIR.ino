#include <SD.h>
#include <M5Unified.h>
#include <M5_ADXL355.h>
#include <SimpleFTPServer.h>   // [U3] FTP server for "Data Dump" mode (xreef/SimpleFTPServer)

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

unsigned int hz = 100;
unsigned int dtWrite = 1000 / hz; //delta T msec
unsigned int SDWriteTime = 15; //sec

// [B5] dt は両タスクから共有されるため portMUX で保護する
auto dt = M5.Rtc.getDateTime();
portMUX_TYPE dtMux = portMUX_INITIALIZER_UNLOCKED;   // [B5] dt 用のクリティカルセクション

char filePath[10];
char fileName[30];
int fileDate = 0;
int fileDateTime;
File f;

// [B14] typo: "begginig" -> "beginning" 等の修正は描画文字列で行う
const String accHeader = "Time(msec),x(cm/s2),y(cm/s2),z(cm/s2)";
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

// [C6 重量版] accData の初期容量。1 行 ~30 byte * (hz*SDWriteTime) 行 + マージン
const size_t ACCDATA_RESERVE = 60000;

// [B4] 送信ドロップ件数を LCD に表示するためのカウンタ
volatile uint32_t dropCount = 0;
volatile uint32_t batchOkCount = 0;

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

// [U3] FTP サーバインスタンス (Data Dump モードで使用)
FtpServer ftpSrv;

//==============================================================================

void txtWrite(const char *string, uint16_t color) {
    M5.Lcd.fillScreen(color);
    M5.Lcd.setTextSize(1);
    // [B10] 文字背景も引数 color に揃える(将来修正予定。現状は黒固定でも判読可)
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
  f = SD.open(fileName, FILE_WRITE);
  if (!f) {
    Serial.print("ERROR: Can't open the file");
    while (1) ;    // [B11] 既知の永久ハング(本修正対象外)
  }
  f.println(accHeader);
}
//==============================================================================

// [U4] WiFi 接続 + NTP 同期にタイムアウトを付与し、失敗を許容する
//      戻り値: true=同期成功, false=失敗(WiFi 圏外等)
bool tryNtpSync(uint32_t wifiTimeoutMs = 15000,
                uint32_t sntpTimeoutMs = 15000) {
  WiFi.begin();
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - t0 > wifiTimeoutMs) {
      WiFi.disconnect(true);
      return false;                              // WiFi 接続失敗
    }
    delay(200);
  }
  configTzTime(NTP_TIMEZONE, NTP_SERVER1, NTP_SERVER2, NTP_SERVER3);
  t0 = millis();
  #if SNTP_ENABLED
    while (sntp_get_sync_status() != SNTP_SYNC_STATUS_COMPLETED) {
      if (millis() - t0 > sntpTimeoutMs) {
        WiFi.disconnect(true);
        return false;                            // NTP 同期失敗
      }
      delay(500);
    }
  #else
    delay(1600);
    struct tm timeInfo;
    if (!getLocalTime(&timeInfo, sntpTimeoutMs)) {
      WiFi.disconnect(true);
      return false;
    }
  #endif

  time_t t = time(nullptr) + 1;
  while (t > time(nullptr));
  M5.Rtc.setDateTime(localtime(&t));
  WiFi.disconnect(true);
  return true;
}

//==============================================================================
void Set_RTC() {
  M5.Lcd.fillScreen(WHITE);
  M5.Lcd.setCursor(0,0);
  M5.Lcd.setTextColor(BLACK, WHITE);
  if (!M5.Rtc.isEnabled())
  {
    M5.Lcd.println("RTC not found.");
    delay(500);
  }
  M5.Lcd.fillScreen(WHITE);
  M5.Lcd.setCursor(0,0);
  M5.Lcd.println("RTC found.");
  M5.Lcd.println("Trying NTP sync...");
  delay(500);

  // [U4] タイムアウト付き NTP 同期。失敗してもハングしない
  bool ok = tryNtpSync();
  if (!ok) {
    M5.Lcd.fillScreen(RED);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.setTextColor(WHITE, RED);
    M5.Lcd.println("NTP sync FAILED.");
    M5.Lcd.println("Continue with");
    M5.Lcd.println("current RTC.");
    delay(2000);
    return;
  }

  // Show 10sec
  M5.Lcd.fillScreen(WHITE);
  for (int i = 100; i > 0; --i) {
    static constexpr const char* const wd[7] = {"Sun","Mon","Tue","Wed","Thr","Fri","Sat"};
    delay(100);
    auto dtNow = M5.Rtc.getDateTime();
    M5.Lcd.setCursor(0,0);
    M5.Lcd.setTextColor(BLACK, WHITE);
    M5.Lcd.printf("RTC   : %04d/%02d/%02d (%s) %02d: %02d: %02d"
                , dtNow.date.year
                , dtNow.date.month
                , dtNow.date.date
                , wd[dtNow.date.weekDay]
                , dtNow.time.hours
                , dtNow.time.minutes
                , dtNow.time.seconds
                );
    auto t = time(nullptr);
    auto tm = localtime(&t);
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
void Set_WiFi(){
  M5.Lcd.fillScreen(WHITE);
  M5.Lcd.setCursor(10, 10);
  M5.Lcd.setTextColor(BLACK, WHITE);
  M5.Lcd.println("Use your phone to select Wi-Fi.");

  WiFi.mode(WIFI_AP_STA);
  WiFi.beginSmartConfig();
  M5.Lcd.setCursor(10, 30);
  M5.Lcd.println("Waiting for SmartConfig.");
  while (!WiFi.smartConfigDone()) {
    delay(500);
  }
  Set_RTC();
}
//==============================================================================

// [U2] 手動時刻設定 UI
//      画面に Y/M/D/h/m/s を一覧表示、各フィールドの上下ボタンで +/-
//      [SET] で M5.Rtc.setDateTime に書き込み
void Manual_Set() {
  // 現在の RTC 値を取得して開始値とする
  auto cur = M5.Rtc.getDateTime();
  int year   = cur.date.year;
  int month  = cur.date.month;
  int day    = cur.date.date;
  int hour   = cur.time.hours;
  int minute = cur.time.minutes;
  int second = cur.time.seconds;

  // ボタン配置(320x240 を想定): 6 列 x (上ボタン / 値 / 下ボタン) + 下段に SET/CANCEL
  // 列幅 50px (offset 10px から 6 列 = 60..360 を 320 内に収める)
  const int colW = 50;
  const int xs[6] = {10, 60, 110, 160, 210, 260};
  const int upY  = 40;
  const int dnY  = 140;
  const int btnH = 40;

  auto drawAll = [&](){
    M5.Lcd.fillScreen(WHITE);
    M5.Lcd.setTextColor(BLACK, WHITE);
    M5.Lcd.setTextFont(2);

    // 上ボタン
    for (int i = 0; i < 6; i++) {
      M5.Lcd.fillRoundRect(xs[i], upY, colW-4, btnH, 5, ORANGE);
      M5.Lcd.setCursor(xs[i] + (colW-4)/2 - 5, upY + btnH/2 - 8);
      M5.Lcd.print("+");
    }
    // 下ボタン
    for (int i = 0; i < 6; i++) {
      M5.Lcd.fillRoundRect(xs[i], dnY, colW-4, btnH, 5, ORANGE);
      M5.Lcd.setCursor(xs[i] + (colW-4)/2 - 5, dnY + btnH/2 - 8);
      M5.Lcd.print("-");
    }
    // ヘッダ
    const char *hdr[6] = {"Year", "Mon", "Day", "Hour", "Min", "Sec"};
    for (int i = 0; i < 6; i++) {
      M5.Lcd.setCursor(xs[i] + 4, 20);
      M5.Lcd.print(hdr[i]);
    }
    // 値表示
    M5.Lcd.setTextFont(4);
    char buf[8];
    int vals[6] = {year, month, day, hour, minute, second};
    for (int i = 0; i < 6; i++) {
      sprintf(buf, "%02d", vals[i] % 100);  // 月日時分秒は 2 桁
      if (i == 0) sprintf(buf, "%04d", vals[i]); // 年は 4 桁(枠は狭いので一部はみ出す)
      M5.Lcd.fillRect(xs[i], 90, colW-4, 40, WHITE);
      M5.Lcd.setCursor(xs[i] + 4, 95);
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

  // フィールド値を加減
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

  // 入力ループ
  while (true) {
    M5.update();
    auto t = M5.Touch.getDetail();
    if (t.wasPressed()) {
      int xt = t.x;
      int yt = t.y;

      // 上ボタン判定
      for (int i = 0; i < 6; i++) {
        if (xt >= xs[i] && xt <= xs[i] + colW - 4 &&
            yt >= upY  && yt <= upY  + btnH) {
          adjust(i, +1);
          drawAll();
        }
      }
      // 下ボタン判定
      for (int i = 0; i < 6; i++) {
        if (xt >= xs[i] && xt <= xs[i] + colW - 4 &&
            yt >= dnY  && yt <= dnY  + btnH) {
          adjust(i, -1);
          drawAll();
        }
      }
      // SET ボタン
      if (xt >= 20 && xt <= 150 && yt >= 195 && yt <= 235) {
        // RTC に書き込み
        m5::rtc_datetime_t newdt;
        newdt.date.year   = year;
        newdt.date.month  = month;
        newdt.date.date   = day;
        newdt.time.hours  = hour;
        newdt.time.minutes= minute;
        newdt.time.seconds= second;
        M5.Rtc.setDateTime(&newdt);

        M5.Lcd.fillScreen(GREEN);
        M5.Lcd.setTextColor(BLACK, GREEN);
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.printf("RTC set:\n%04d/%02d/%02d %02d:%02d:%02d",
                      year, month, day, hour, minute, second);
        delay(2000);
        return;
      }
      // CANCEL ボタン
      if (xt >= 170 && xt <= 300 && yt >= 195 && yt <= 235) {
        return;
      }
    }
    delay(20);
  }
}

//==============================================================================

// [U3] Data Dump モード: SoftAP を立て、FTP サーバ経由で SD 内容を PC に提供
//      Explorer のアドレスバーに ftp://m5:m5@192.168.4.1 で接続可能
void Data_Dump_FTP() {
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextFont(2);
  M5.Lcd.println("Data Dump (AP+FTP)");
  M5.Lcd.println();

  // SoftAP 起動
  WiFi.mode(WIFI_AP);
  IPAddress apIP(192, 168, 4, 1);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP("M5-SEISMO", "m5seismo");

  // FTP サーバ起動 (user/pass は m5/m5)
  ftpSrv.begin("m5", "m5");

  // 案内表示
  M5.Lcd.printf("SSID: M5-SEISMO\n");
  M5.Lcd.printf("PASS: m5seismo\n");
  M5.Lcd.println();
  M5.Lcd.printf("URL : ftp://192.168.4.1\n");
  M5.Lcd.printf("User: m5\n");
  M5.Lcd.printf("Pass: m5\n");
  M5.Lcd.println();
  M5.Lcd.println("Open in Explorer:");
  M5.Lcd.println("ftp://m5:m5@192.168.4.1");
  M5.Lcd.println();
  M5.Lcd.println("Reset M5 to exit.");

  // FTP イベントループ(リセットまで戻らない)
  while (true) {
    ftpSrv.handleFTP();
    delay(1);
  }
}

//==============================================================================

void setup() {
  // Initialize M5
  auto cfg = M5.config();
  cfg.serial_baudrate = 921600;          // [U3] Data Dump とは無関係だが、シリアル高速化
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
  // [U1] 起動メニュー: 4 ボタン
  //   1) Wi-Fi Setting (SmartConfig)
  //   2) Reset RTC! (NTP 同期、タイムアウト付き)
  //   3) Manual Set (手動時刻設定)
  //   4) Data Dump (SoftAP + FTP)
  M5.Lcd.fillScreen(WHITE);
  const int btnX = 60;
  const int btnW = 200;
  const int btnH = 40;        // 4 ボタン収納のため、従来 50 -> 40 に縮小
  const int btnGap = 5;       // 同上、間隔も 10 -> 5
  const int btnR = 8;
  const int btnY0 = 40;       // 1 つ目の y 座標

  struct Btn { const char *label; int y; };
  Btn btns[4] = {
    {"Wi-Fi Setting", btnY0 + 0 * (btnH + btnGap)},
    {"Reset RTC!",    btnY0 + 1 * (btnH + btnGap)},
    {"Manual Set",    btnY0 + 2 * (btnH + btnGap)},
    {"Data Dump",     btnY0 + 3 * (btnH + btnGap)},
  };
  // ボタン描画
  for (int i = 0; i < 4; i++) {
    M5.Lcd.fillRoundRect(btnX,     btns[i].y,     btnW,     btnH,     btnR, RED);
    M5.Lcd.fillRoundRect(btnX + 4, btns[i].y + 4, btnW - 8, btnH - 8, btnR, ORANGE);
    M5.Lcd.setTextFont(2);
    M5.Lcd.setTextColor(BLACK, ORANGE);
    M5.Lcd.setCursor(btnX + 30, btns[i].y + 10);
    M5.Lcd.print(btns[i].label);
  }
  M5.Lcd.setTextColor(BLACK, WHITE);

  // 30 秒カウントダウン (タップ無しでスキップ)
  // [B7] 100 ループ x 100ms = 約 10 秒。表示の "30 sec" は誤記なので "Wait..." に変更
  for (int i = 100; i > 0; --i) {
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
            case 3: Data_Dump_FTP(); break;  // 戻ってこない
          }
          dispatched = true;
          break;
        }
      }
    }
    if (dispatched) break;

    M5.Lcd.setCursor(0, 0);
    M5.Lcd.setTextFont(2);
    M5.Lcd.setTextColor(BLACK, WHITE);
    M5.Lcd.print("Tap a button:");
    M5.Lcd.setCursor(0, 220);
    M5.Lcd.printf("Wait... %3d", i);
    delay(100);             // [B7] 表示と実時間を一致させるため delay 追加
  }
  //==============================================================================

  // Initialize ADXL355
  adxl355.begin();
  adxl355.setRange(range);
  adxl355.setOutputDataRate(ODR);
  adxl355.setHpfFrequency(HPF);
  adxl355.setSynchronization(syncTime);
  adxl355.enableMeasurement();

  // [C7] FIR 係数の和を確認(DC ゲイン = 係数和)。1.0 から大きく外れていれば警告
  double firSum = 0.;
  for (int k = 0; k < BufNum; k++) firSum += FIR[k];
  Serial.printf("FIR coef sum = %.6f\n", firSum);
  if (fabs(firSum - 1.0) > 0.01) {     // 1% 以上ズレなら LCD でも警告
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

  // [B14] typo: "Strat RTC" -> "Start RTC", "begginig" -> "beginning"
  dt = M5.Rtc.getDateTime();
  txtWrite("Start RTC. waiting...", BLACK);
  while ((dt.date.year < 2026) || (dt.date.year > 2099)) {  // [B8] 上限緩和: 2031 -> 2099
    Serial.println(dt.date.year);
    delay(10);
    dt = M5.Rtc.getDateTime();
  }

  // [B1] タスク生成前に最初のファイルを開いておく(初回バッチ消失防止)
  dt = M5.Rtc.getDateTime();
  createPath();
  createFile();

  // [B2] キューはポインタ(String*)を 3 個まで保持。SD 遅延を吸収
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
  accData.reserve(ACCDATA_RESERVE);   // [C6 重量版] 初回 realloc を抑制

  // [B14] typo: "waite starting..." -> "waiting start..."
  m5::rtc_datetime_t localDt;
  localDt = M5.Rtc.getDateTime();
  txtWrite("waiting start...", BLACK);
  while (localDt.time.seconds != 0) {
    delay(1);
    localDt = M5.Rtc.getDateTime();
  }
  // [B5] dt 更新はクリティカルセクション内で
  portENTER_CRITICAL(&dtMux);
  dt = localDt;
  portEXIT_CRITICAL(&dtMux);

  TickType_t xLastWakeTimeSend = xTaskGetTickCount();    // [B12] portTickType -> TickType_t

  for (;;) {
    while (j <= dtWrite / dtRead) {
      // [B12] portTICK_RATE_MS -> portTICK_PERIOD_MS
      vTaskDelayUntil(&xLastWakeTimeSend, dtRead / portTICK_PERIOD_MS);
      sTime = millis();
      accelerations = adxl355.getAccelerations();
      AccX = accelerations.x;
      AccY = accelerations.y;
      AccZ = accelerations.z;
      if (BufIndex >= BufNum) BufIndex = 0;
      RingBufX[BufIndex] = AccX;
      RingBufY[BufIndex] = AccY;
      RingBufZ[BufIndex] = AccZ;
      BufIndex++;
      j++;
    }
    j = 1;

    // FIR Filter & decimation
    l = BufIndex - 1;
    for (k = 0; k < (unsigned int)BufNum; k++) {
      if (l < k) l = BufNum - 1 + k;
      AccFirX += FIR[k] * RingBufX[l - k];
      AccFirY += FIR[k] * RingBufY[l - k];
      AccFirZ += FIR[k] * RingBufZ[l - k];
    }

    // [C6 軽量版] 一時 String を作らずに直接 += する
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

    AccFirX = 0;
    AccFirY = 0;
    AccFirZ = 0;

    if (i >= hz * SDWriteTime) {
      // [B2] String* をヒープに deep-copy して送る。バッファ共有レースを防止
      String *snapshot = new String(accData);
      // [B4] 送信失敗 (キュー満杯) は delete してドロップ件数をカウント
      if (xQueueSendToBack(xQueue, &snapshot, 0) != pdTRUE) {
        delete snapshot;
        dropCount++;
      }
      accData = "";                 // 内部 buffer は再利用、capacity 維持
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

  delay(5);

  // [C5] LCD 部分更新の状態保持。前回値と異なる時だけ書き換える
  char prevTime[32] = "";
  uint32_t prevOk = 0xFFFFFFFFu;
  uint32_t prevDrop = 0xFFFFFFFFu;

  for (;;) {
    // [B3] xQueueReceive の戻り値を確認。タイムアウト時は書き込みをスキップ
    if (xQueueReceive(xQueue, &recData, SDWriteTime * 1000 / portTICK_PERIOD_MS) != pdTRUE) {
      // タイムアウト: 時計表示だけ更新して継続
      m5::rtc_datetime_t localDt = M5.Rtc.getDateTime();
      portENTER_CRITICAL(&dtMux);
      dt = localDt;                                          // [B5]
      portEXIT_CRITICAL(&dtMux);
      continue;
    }

    *recData += '\n';                              // [C6] 一時 String 不要
    f.print(recData->substring(1));
    f.close();
    batchOkCount++;

    // [C5] 全画面 fillScreen を廃止し、変化フィールドだけ重ね描画
    m5::rtc_datetime_t localDt = M5.Rtc.getDateTime();
    portENTER_CRITICAL(&dtMux);
    dt = localDt;                                            // [B5]
    portEXIT_CRITICAL(&dtMux);

    char nowStr[32];
    sprintf(nowStr, "%04d/%02d/%02d %02d:%02d:%02d",
            localDt.date.year, localDt.date.month, localDt.date.date,
            localDt.time.hours, localDt.time.minutes, localDt.time.seconds);

    M5.Lcd.setTextFont(4);
    M5.Lcd.setTextColor(WHITE, BLACK);
    if (strcmp(nowStr, prevTime) != 0) {
      M5.Lcd.fillRect(0, 0, 320, 30, BLACK);     // 時刻表示エリアのみクリア
      M5.Lcd.setCursor(0, 0);
      M5.Lcd.print(nowStr);
      strcpy(prevTime, nowStr);
    }
    if (batchOkCount != prevOk) {
      M5.Lcd.fillRect(0, 40, 320, 30, BLACK);
      M5.Lcd.setCursor(0, 40);
      M5.Lcd.printf("OK : %lu", (unsigned long)batchOkCount);
      prevOk = batchOkCount;
    }
    if (dropCount != prevDrop) {
      // [B4] ドロップ件数表示 (0 でなければ赤字で警告)
      M5.Lcd.fillRect(0, 70, 320, 30, BLACK);
      M5.Lcd.setTextColor(dropCount ? RED : WHITE, BLACK);
      M5.Lcd.setCursor(0, 70);
      M5.Lcd.printf("DROP: %lu", (unsigned long)dropCount);
      M5.Lcd.setTextColor(WHITE, BLACK);
      prevDrop = dropCount;
    }

    delete recData;     // [B2] 受信した String* を解放
    recData = nullptr;

    // 分が変わったら新規ファイル、変わらなければ追記でオープン
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
      f = SD.open(fileName, FILE_APPEND);
    }
  }
}
