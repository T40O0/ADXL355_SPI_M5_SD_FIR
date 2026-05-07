#include <SD.h>
#include <M5Unified.h>
#include <M5_ADXL355.h>

// [U3] HTTP file server for "Data Dump" mode
//      ESP32 標準の WebServer.h のみ使用 (外部ライブラリ不要、自動更新の懸念なし)
//      個別 DL + 一括 ZIP DL に対応 (ZIP は無圧縮 store 形式でストリーミング生成)
#include <WebServer.h>
#include <vector>

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

// [B4] 送信ドロップ件数(欠測バッチ数) と OK 件数 (LCD 表示用)
//      DROP は SD 書き込みが追いつかずキューが満杯になった時に発生する欠測数
volatile uint32_t dropCount = 0;
volatile uint32_t batchOkCount = 0;

// [C5] 最新の FIR 出力 (LCD で振動データ確認用)
//      TaskRead が更新、TaskSave が読む。並行アクセスは表示用なので保護不要
double latestX = 0.0;
double latestY = 0.0;
double latestZ = 0.0;

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

// [U3] HTTP サーバインスタンス (Data Dump モードで使用、ポート 80)
WebServer httpSrv(80);

// [U3] CRC32 テーブル (ZIP 用、起動時に計算)
static uint32_t crc32Table[256];
static bool crc32TableReady = false;
static void crc32Init() {
  if (crc32TableReady) return;
  for (int i = 0; i < 256; i++) {
    uint32_t c = i;
    for (int j = 0; j < 8; j++) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
    crc32Table[i] = c;
  }
  crc32TableReady = true;
}
static uint32_t crc32Update(uint32_t crc, const uint8_t *data, size_t len) {
  crc = ~crc;
  while (len--) crc = crc32Table[(crc ^ *data++) & 0xFF] ^ (crc >> 8);
  return ~crc;
}

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

// [U4 削除] tryNtpSync (タイムアウト版) は元のハング動作に戻すため削除

// [U2/Set_RTC 共通] 設定後の確認画面 (10 秒表示、RTC と ESP32 内部時計を比較)
//      フォント大きめで読みやすく改善
static void rtcConfirmScreen() {
  static constexpr const char* const wd[7] = {"Sun","Mon","Tue","Wed","Thr","Fri","Sat"};
  M5.Lcd.fillScreen(WHITE);
  for (int i = 100; i > 0; --i) {
    delay(100);
    auto rtcDt = M5.Rtc.getDateTime();
    auto sysT  = time(nullptr);
    auto sysTm = localtime(&sysT);

    // RTC 時刻 (font 4)
    M5.Lcd.setTextFont(2);
    M5.Lcd.setTextColor(BLACK, WHITE);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.print("RTC time:");
    M5.Lcd.setTextFont(4);
    M5.Lcd.fillRect(0, 18, 320, 30, WHITE);
    M5.Lcd.setCursor(0, 18);
    M5.Lcd.printf("%04d/%02d/%02d (%s)",
                  rtcDt.date.year, rtcDt.date.month, rtcDt.date.date, wd[rtcDt.date.weekDay]);
    M5.Lcd.fillRect(0, 50, 320, 30, WHITE);
    M5.Lcd.setCursor(0, 50);
    M5.Lcd.printf("%02d:%02d:%02d",
                  rtcDt.time.hours, rtcDt.time.minutes, rtcDt.time.seconds);

    // ESP32 内部時計 (font 4)
    M5.Lcd.setTextFont(2);
    M5.Lcd.setCursor(0, 90);
    M5.Lcd.print("ESP32 time:");
    M5.Lcd.setTextFont(4);
    M5.Lcd.fillRect(0, 108, 320, 30, WHITE);
    M5.Lcd.setCursor(0, 108);
    M5.Lcd.printf("%04d/%02d/%02d (%s)",
                  sysTm->tm_year+1900, sysTm->tm_mon+1, sysTm->tm_mday, wd[sysTm->tm_wday]);
    M5.Lcd.fillRect(0, 140, 320, 30, WHITE);
    M5.Lcd.setCursor(0, 140);
    M5.Lcd.printf("%02d:%02d:%02d",
                  sysTm->tm_hour, sysTm->tm_min, sysTm->tm_sec);

    // カウントダウン (font 2)
    M5.Lcd.setTextFont(2);
    M5.Lcd.fillRect(0, 200, 320, 20, WHITE);
    M5.Lcd.setCursor(0, 200);
    M5.Lcd.printf("Measurement starts in %d sec", i / 10);
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

  // [NTP 同期] 元のハング動作に戻す: 圏外なら永久に WiFi 接続待ち / NTP 同期待ち
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
  // [U2 修正] 初期値は固定で 2026/01/01 00:00:00 (RTC の現在値は使わない)
  int year   = 2026;
  int month  = 1;
  int day    = 1;
  int hour   = 0;
  int minute = 0;
  int second = 0;

  // ボタン配置(320x240):
  //   年欄 70px (4 桁)、月日時分秒 各 50px (2 桁、原版と同じ幅)
  //   [Year=70][Mon=50][Day=50][Hour=50][Min=50][Sec=40] = 310px (左マージン 5px)
  const int xs[6]   = {   5,  80, 132, 184, 236, 288 };
  const int colW[6] = {  70,  48,  48,  48,  48,  30 };
  // [U2 修正] SET ボタン誤タップ防止のため、Down ボタンを上に移動 (ギャップ 35px 確保)
  const int upY  = 30;
  const int dnY  = 120;
  const int btnH = 40;

  auto drawAll = [&](){
    M5.Lcd.fillScreen(WHITE);
    M5.Lcd.setTextColor(BLACK, WHITE);
    M5.Lcd.setTextFont(2);

    // 上ボタン
    for (int i = 0; i < 6; i++) {
      M5.Lcd.fillRoundRect(xs[i], upY, colW[i], btnH, 5, ORANGE);
      M5.Lcd.setCursor(xs[i] + colW[i]/2 - 5, upY + btnH/2 - 8);
      M5.Lcd.setTextColor(BLACK, ORANGE);
      M5.Lcd.print("+");
    }
    // 下ボタン
    for (int i = 0; i < 6; i++) {
      M5.Lcd.fillRoundRect(xs[i], dnY, colW[i], btnH, 5, ORANGE);
      M5.Lcd.setCursor(xs[i] + colW[i]/2 - 5, dnY + btnH/2 - 8);
      M5.Lcd.setTextColor(BLACK, ORANGE);
      M5.Lcd.print("-");
    }
    // ヘッダ (列名)
    M5.Lcd.setTextColor(BLACK, WHITE);
    const char *hdr[6] = {"Year", "Mon", "Day", "Hour", "Min", "Sec"};
    for (int i = 0; i < 6; i++) {
      M5.Lcd.setCursor(xs[i] + 4, 5);     // [U2 修正] 上端寄せ (上ボタンと重ならない)
      M5.Lcd.print(hdr[i]);
    }
    // 値表示 (年は 4 桁、他は 2 桁) 上下ボタンの間 (y=80 付近)
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
        if (xt >= xs[i] && xt <= xs[i] + colW[i] &&
            yt >= upY  && yt <= upY  + btnH) {
          adjust(i, +1);
          drawAll();
        }
      }
      // 下ボタン判定
      for (int i = 0; i < 6; i++) {
        if (xt >= xs[i] && xt <= xs[i] + colW[i] &&
            yt >= dnY  && yt <= dnY  + btnH) {
          adjust(i, -1);
          drawAll();
        }
      }
      // SET ボタン
      if (xt >= 20 && xt <= 150 && yt >= 195 && yt <= 235) {
        // RTC に書き込み (auto で型名依存を回避)
        auto newdt = M5.Rtc.getDateTime();
        newdt.date.year   = year;
        newdt.date.month  = month;
        newdt.date.date   = day;
        newdt.time.hours  = hour;
        newdt.time.minutes= minute;
        newdt.time.seconds= second;
        M5.Rtc.setDateTime(&newdt);

        // ESP32 内部時計も同期しておく(rtcConfirmScreen で照合表示するため)
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

        // [U2] Set_RTC と共通の確認画面 (10 秒、フォント大)
        rtcConfirmScreen();
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

// [U3] HTTP 経由でファイル一覧を返すユーティリティ
//      指定ディレクトリ内の名前を String 配列に集める (ファイルとディレクトリ別)
static void listDir(const String &path, std::vector<String> &outFiles, std::vector<String> &outDirs) {
  File dir = SD.open(path);
  if (!dir || !dir.isDirectory()) return;
  while (true) {
    File entry = dir.openNextFile();
    if (!entry) break;
    String name = entry.name();
    // [U3] SD ライブラリは name に絶対パスを返す場合があるので basename だけ抽出
    int slash = name.lastIndexOf('/');
    if (slash >= 0) name = name.substring(slash + 1);
    if (entry.isDirectory()) outDirs.push_back(name);
    else                     outFiles.push_back(name);
    entry.close();
  }
  dir.close();
}

// [U3] HTTP ルート: トップレベルのフォルダ/ファイル一覧
static void handleRoot() {
  std::vector<String> files, dirs;
  listDir("/", files, dirs);

  String html;
  html.reserve(8192);
  html  = F("<!doctype html><html><head><meta charset=\"utf-8\"><title>M5-SEISMO Data</title>");
  html += F("<style>body{font-family:sans-serif;margin:20px}a{text-decoration:none}li{margin:4px 0}</style></head><body>");
  html += F("<h1>M5-SEISMO Data Server</h1>");
  html += F("<p><a href=\"/zipall\">[Download ALL as ZIP]</a></p>");
  html += F("<h2>Folders</h2><ul>");
  for (auto &d : dirs) {
    html += "<li><a href=\"/folder?p=/" + d + "\">" + d + "/</a> ";
    html += "[<a href=\"/zip?p=/" + d + "\">ZIP</a>]</li>";
  }
  html += F("</ul><h2>Files (root)</h2><ul>");
  for (auto &f : files) {
    html += "<li><a href=\"/dl?p=/" + f + "\">" + f + "</a></li>";
  }
  html += F("</ul></body></html>");
  httpSrv.send(200, "text/html; charset=utf-8", html);
}

// [U3] HTTP /folder: 指定フォルダ内のファイル一覧
static void handleFolder() {
  String p = httpSrv.arg("p");
  if (p.length() == 0 || p[0] != '/') { httpSrv.send(400, "text/plain", "bad path"); return; }
  std::vector<String> files, dirs;
  listDir(p, files, dirs);

  String html;
  html.reserve(16384);
  html  = F("<!doctype html><html><head><meta charset=\"utf-8\"><title>");
  html += p;
  html += F("</title></head><body>");
  html += "<h1>" + p + "</h1>";
  html += "<p><a href=\"/\">&larr; Back</a> &nbsp; <a href=\"/zip?p=" + p + "\">[Download this folder as ZIP]</a></p>";
  html += F("<ul>");
  for (auto &f : files) {
    html += "<li><a href=\"/dl?p=" + p + "/" + f + "\">" + f + "</a></li>";
  }
  html += F("</ul></body></html>");
  httpSrv.send(200, "text/html; charset=utf-8", html);
}

// [U3] HTTP /dl: 個別ファイル DL
static void handleDownload() {
  String p = httpSrv.arg("p");
  if (p.length() == 0 || p[0] != '/') { httpSrv.send(400, "text/plain", "bad path"); return; }
  File f = SD.open(p);
  if (!f || f.isDirectory()) { httpSrv.send(404, "text/plain", "not found"); if (f) f.close(); return; }
  // basename を取り出して Content-Disposition に
  int slash = p.lastIndexOf('/');
  String fname = (slash >= 0) ? p.substring(slash + 1) : p;
  httpSrv.sendHeader("Content-Disposition", "attachment; filename=\"" + fname + "\"");
  httpSrv.streamFile(f, "text/csv");
  f.close();
}

// [U3] ZIP ストリーミング本体: ファイルパスのリストを ZIP として送出
//      無圧縮 (store) 形式。ZIP-32 (4GB 以下) 対応
//      [U3 修正] httpSrv.sendContent() を使う (chunked encoding を WebServer に任せる)
//                生 client.write() だと CONTENT_LENGTH_UNKNOWN 時の chunk フレーミングが
//                付かず HTTP 不正でブラウザがエラー → DL 失敗する
static void streamZip(const std::vector<String> &fullPaths, const std::vector<String> &archiveNames) {
  crc32Init();
  String centralDir;
  centralDir.reserve(fullPaths.size() * 80);
  uint32_t totalOffset = 0;

  uint8_t hdr[46];
  uint8_t buf[1024];

  for (size_t idx = 0; idx < fullPaths.size(); idx++) {
    File f = SD.open(fullPaths[idx]);
    if (!f || f.isDirectory()) { if (f) f.close(); continue; }
    uint32_t size = f.size();

    // CRC32 を計算 (ファイルを 1 周読む)
    uint32_t crc = 0;
    while (f.available()) {
      int n = f.read(buf, sizeof(buf));
      if (n <= 0) break;
      crc = crc32Update(crc, buf, n);
    }
    f.seek(0);

    const String &name = archiveNames[idx];
    uint16_t nameLen = name.length();

    // Local file header
    memset(hdr, 0, 30);
    hdr[0]=0x50; hdr[1]=0x4b; hdr[2]=0x03; hdr[3]=0x04;
    hdr[4]=20;
    hdr[12]=0x21;
    hdr[14]= crc        & 0xFF; hdr[15]=(crc>>8)&0xFF; hdr[16]=(crc>>16)&0xFF; hdr[17]=(crc>>24)&0xFF;
    hdr[18]= size       & 0xFF; hdr[19]=(size>>8)&0xFF; hdr[20]=(size>>16)&0xFF; hdr[21]=(size>>24)&0xFF;
    hdr[22]= size       & 0xFF; hdr[23]=(size>>8)&0xFF; hdr[24]=(size>>16)&0xFF; hdr[25]=(size>>24)&0xFF;
    hdr[26]= nameLen    & 0xFF; hdr[27]=(nameLen>>8)&0xFF;
    httpSrv.sendContent((const char*)hdr, 30);
    httpSrv.sendContent(name);

    // ファイル本体ストリーミング
    while (f.available()) {
      int n = f.read(buf, sizeof(buf));
      if (n <= 0) break;
      httpSrv.sendContent((const char*)buf, n);
    }
    f.close();

    // Central directory entry をバッファ追記
    uint8_t cd[46];
    memset(cd, 0, 46);
    cd[0]=0x50; cd[1]=0x4b; cd[2]=0x01; cd[3]=0x02;
    cd[4]=20; cd[6]=20;
    cd[12]=0x21;
    cd[16]= crc        & 0xFF; cd[17]=(crc>>8)&0xFF; cd[18]=(crc>>16)&0xFF; cd[19]=(crc>>24)&0xFF;
    cd[20]= size       & 0xFF; cd[21]=(size>>8)&0xFF; cd[22]=(size>>16)&0xFF; cd[23]=(size>>24)&0xFF;
    cd[24]= size       & 0xFF; cd[25]=(size>>8)&0xFF; cd[26]=(size>>16)&0xFF; cd[27]=(size>>24)&0xFF;
    cd[28]= nameLen    & 0xFF; cd[29]=(nameLen>>8)&0xFF;
    cd[42]= totalOffset & 0xFF; cd[43]=(totalOffset>>8)&0xFF; cd[44]=(totalOffset>>16)&0xFF; cd[45]=(totalOffset>>24)&0xFF;
    centralDir.concat((const char*)cd, 46);
    centralDir.concat(name);

    totalOffset += 30 + nameLen + size;
  }

  // Central directory 全体を送信
  uint32_t cdSize = centralDir.length();
  uint32_t cdOffset = totalOffset;
  if (cdSize > 0) {
    httpSrv.sendContent(centralDir.c_str(), cdSize);
  }

  // End of Central Directory record
  uint8_t eocd[22];
  memset(eocd, 0, 22);
  eocd[0]=0x50; eocd[1]=0x4b; eocd[2]=0x05; eocd[3]=0x06;
  uint16_t entries = (uint16_t)fullPaths.size();
  eocd[8]=  entries  & 0xFF; eocd[9]=(entries>>8)&0xFF;
  eocd[10]= entries  & 0xFF; eocd[11]=(entries>>8)&0xFF;
  eocd[12]= cdSize   & 0xFF; eocd[13]=(cdSize>>8)&0xFF; eocd[14]=(cdSize>>16)&0xFF; eocd[15]=(cdSize>>24)&0xFF;
  eocd[16]= cdOffset & 0xFF; eocd[17]=(cdOffset>>8)&0xFF; eocd[18]=(cdOffset>>16)&0xFF; eocd[19]=(cdOffset>>24)&0xFF;
  httpSrv.sendContent((const char*)eocd, 22);

  // chunked 終端 (空 chunk)
  httpSrv.sendContent("");
}

// [U3] /zip?p=<folder>: 指定フォルダ内の全ファイルを ZIP で DL
static void handleZipFolder() {
  String p = httpSrv.arg("p");
  if (p.length() == 0 || p[0] != '/') { httpSrv.send(400, "text/plain", "bad path"); return; }

  std::vector<String> files, dirs;
  listDir(p, files, dirs);
  // フォルダ名を ZIP ファイル名に
  String zipName = p.substring(1) + ".zip";

  std::vector<String> fullPaths, archiveNames;
  for (auto &f : files) {
    fullPaths.push_back(p + "/" + f);
    archiveNames.push_back(f);   // フォルダ名は付けず flat に格納
  }

  httpSrv.sendHeader("Content-Type", "application/zip");
  httpSrv.sendHeader("Content-Disposition", "attachment; filename=\"" + zipName + "\"");
  httpSrv.sendHeader("Connection", "close");
  httpSrv.setContentLength(CONTENT_LENGTH_UNKNOWN);
  httpSrv.send(200, "application/zip", "");
  streamZip(fullPaths, archiveNames);
}

// [U3] /zipall: SD ルート以下の全ファイルを再帰的に ZIP
static void handleZipAll() {
  std::vector<String> fullPaths, archiveNames;
  // ルートをスキャン
  std::vector<String> rootFiles, rootDirs;
  listDir("/", rootFiles, rootDirs);
  for (auto &f : rootFiles) {
    fullPaths.push_back("/" + f);
    archiveNames.push_back(f);
  }
  for (auto &d : rootDirs) {
    std::vector<String> sub, subD;
    listDir("/" + d, sub, subD);
    for (auto &f : sub) {
      fullPaths.push_back("/" + d + "/" + f);
      archiveNames.push_back(d + "/" + f);   // フォルダ名込みで格納
    }
  }

  httpSrv.sendHeader("Content-Type", "application/zip");
  httpSrv.sendHeader("Content-Disposition", "attachment; filename=\"all.zip\"");
  httpSrv.sendHeader("Connection", "close");
  httpSrv.setContentLength(CONTENT_LENGTH_UNKNOWN);
  httpSrv.send(200, "application/zip", "");
  streamZip(fullPaths, archiveNames);
}

// [U3] Data Dump モード: SoftAP + HTTP サーバ
//      ブラウザで http://192.168.4.1 にアクセスしてファイル一覧 / 個別 DL / 一括 ZIP DL
void Data_Dump_HTTP() {
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextFont(2);
  M5.Lcd.println("Data Dump (AP+HTTP)");
  M5.Lcd.println();

  // [U3 修正] Data Dump は setup() 中の startup menu から呼ばれ、
  //          まだ SD.begin() していないため、ここで初期化する
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

  // SoftAP 起動
  WiFi.mode(WIFI_AP);
  IPAddress apIP(192, 168, 4, 1);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP("M5-SEISMO", "m5seismo");

  // [U3] WebServer ルーティング
  httpSrv.on("/",       HTTP_GET, handleRoot);
  httpSrv.on("/folder", HTTP_GET, handleFolder);
  httpSrv.on("/dl",     HTTP_GET, handleDownload);
  httpSrv.on("/zip",    HTTP_GET, handleZipFolder);
  httpSrv.on("/zipall", HTTP_GET, handleZipAll);
  httpSrv.begin();

  M5.Lcd.printf("SSID: M5-SEISMO\n");
  M5.Lcd.printf("PASS: m5seismo\n");
  M5.Lcd.println();
  M5.Lcd.printf("Open in browser:\n");
  M5.Lcd.println("http://192.168.4.1");
  M5.Lcd.println();
  M5.Lcd.println("Click [Download ALL as ZIP]");
  M5.Lcd.println("for full backup.");
  M5.Lcd.println();
  M5.Lcd.println("Reset M5 to exit.");

  while (true) {
    httpSrv.handleClient();
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
  // ボタン描画 (font 4 で読みやすく、太字感のある表示)
  for (int i = 0; i < 4; i++) {
    M5.Lcd.fillRoundRect(btnX,     btns[i].y,     btnW,     btnH,     btnR, RED);
    M5.Lcd.fillRoundRect(btnX + 4, btns[i].y + 4, btnW - 8, btnH - 8, btnR, ORANGE);
    M5.Lcd.setTextFont(4);
    M5.Lcd.setTextColor(BLACK, ORANGE);
    // ラベルを中央寄せ気味に (font 4 は char 幅 ~13px)
    int textX = btnX + (btnW - (int)strlen(btns[i].label) * 13) / 2;
    if (textX < btnX + 5) textX = btnX + 5;
    M5.Lcd.setCursor(textX, btns[i].y + 8);
    M5.Lcd.print(btns[i].label);
  }
  M5.Lcd.setTextColor(BLACK, WHITE);

  // 30 秒カウントダウン (タップ無しでスキップ)
  // [B7] 300 ループ x 100ms = 30 秒
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
            case 3: Data_Dump_HTTP(); break;  // 戻ってこない
          }
          dispatched = true;
          break;
        }
      }
    }
    if (dispatched) break;

    // [U1 改善] font 4 で大きく表示。ヘッダ "Tap a button:" は上部、カウントダウンは下部
    M5.Lcd.setTextFont(4);
    M5.Lcd.setTextColor(BLACK, WHITE);
    M5.Lcd.setCursor(0, 5);
    M5.Lcd.print("Tap a button:");
    M5.Lcd.fillRect(0, 215, 320, 25, WHITE);   // 下端の数字エリアをクリアしてから描画
    M5.Lcd.setCursor(0, 215);
    M5.Lcd.printf("Wait... %3d sec", i / 10);
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
  auto localDt = M5.Rtc.getDateTime();
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

    // [C5] 最新値を LCD 表示用にコピー (リセット前)
    latestX = AccFirX;
    latestY = AccFirY;
    latestZ = AccFirZ;

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

  for (;;) {
    // [B3] xQueueReceive の戻り値を確認。タイムアウト時は書き込みをスキップ
    if (xQueueReceive(xQueue, &recData, SDWriteTime * 1000 / portTICK_PERIOD_MS) != pdTRUE) {
      auto localDt = M5.Rtc.getDateTime();
      portENTER_CRITICAL(&dtMux);
      dt = localDt;                                          // [B5]
      portEXIT_CRITICAL(&dtMux);
      continue;
    }

    *recData += '\n';                              // [C6] 一時 String 不要
    f.print(recData->substring(1));
    f.close();
    batchOkCount++;

    auto localDt = M5.Rtc.getDateTime();
    portENTER_CRITICAL(&dtMux);
    dt = localDt;                                            // [B5]
    portEXIT_CRITICAL(&dtMux);

    // [C5] 焼付き対策: 1 バッチ毎に 3 秒だけ表示し、その後画面消灯
    //      表示内容: 時刻 / 最新 XYZ (振動データ確認用) / OK 件数 / DROP 欠測数 (>0 で赤)
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
    // [B4] DROP は欠測バッチ数 (SD 書き込み遅延でキュー満杯時にカウント)
    M5.Lcd.setCursor(0, 130);
    M5.Lcd.setTextColor(dropCount ? RED : WHITE, BLACK);
    M5.Lcd.printf("DROP: %lu", (unsigned long)dropCount);
    M5.Lcd.setTextColor(WHITE, BLACK);

    // 約 3 秒表示 → 消灯 (15s 周期のうち 3s 点灯、12s 消灯で焼付き軽減)
    delay(SDWriteTime * 1000 / 5);
    M5.Lcd.fillScreen(BLACK);

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
