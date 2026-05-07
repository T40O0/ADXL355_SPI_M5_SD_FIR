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

// [B11] SD オープン失敗時の再試行ヘルパー
//       一時的な接触不良/書き込み遅延を吸収するため、永久 while(1) ;の代わりに
//       1 秒間隔でリトライ + 30 回ごとに SD 再マウント。
//       LCD/Serial に「SD ERROR retry=N」を表示してユーザに気づかせる。
//       戻り値は必ずオープン済みの File (失敗ループは無限に続く)。
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
    // 30 秒ごとに SD を再マウント (ホットプラグ復帰対応)
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
  // [B11] 旧 while(1) を openSDFileSafe で置換: 一時的 SD 障害でハングしない
  f = openSDFileSafe(fileName, FILE_WRITE);
  f.println(accHeader);
}
//==============================================================================

// [U4 削除] tryNtpSync (タイムアウト版) は元のハング動作に戻すため削除

// [U2/Set_RTC 共通] 設定後の確認画面 (10 秒表示、RTC と ESP32 内部時計を比較)
//      [改善] fillRect を廃止 → ちらつき解消
//             ラベルは初回のみ描画。値はフォント背景色で上書き(同じ幅なので残骸なし)。
//             カウントダウンは秒が変わった時だけ再描画。
static void rtcConfirmScreen() {
  static constexpr const char* const wd[7] = {"Sun","Mon","Tue","Wed","Thr","Fri","Sat"};

  // 初回: 全画面塗りつぶし + 静的ラベル描画
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

    // RTC 時刻 (font 4 で大表示、固定幅フォーマット)
    M5.Lcd.setTextFont(4);
    M5.Lcd.setTextColor(BLACK, WHITE);   // 文字背景を WHITE に → fillRect 不要
    M5.Lcd.setCursor(0, 18);
    M5.Lcd.printf("%04d/%02d/%02d (%s)",
                  rtcDt.date.year, rtcDt.date.month, rtcDt.date.date, wd[rtcDt.date.weekDay]);
    M5.Lcd.setCursor(0, 50);
    M5.Lcd.printf("%02d:%02d:%02d",
                  rtcDt.time.hours, rtcDt.time.minutes, rtcDt.time.seconds);

    // ESP32 内部時計
    M5.Lcd.setCursor(0, 108);
    M5.Lcd.printf("%04d/%02d/%02d (%s)",
                  sysTm->tm_year+1900, sysTm->tm_mon+1, sysTm->tm_mday, wd[sysTm->tm_wday]);
    M5.Lcd.setCursor(0, 140);
    M5.Lcd.printf("%02d:%02d:%02d",
                  sysTm->tm_hour, sysTm->tm_min, sysTm->tm_sec);

    // カウントダウン: 秒が変わった時だけ再描画 (10 ループに 1 回)
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
  // [U1 改善] font 4 (1 行 ~22 文字まで) に明示設定 + 文字列を 2 行ずつに分割
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
//      [U3 修正] outSizes を渡せば、各ファイルのサイズも同時に収集する。
//                ZIP 生成では事前のサイズ収集に使う(Pass1 の SD.open 連打を回避)。
static void listDir(const String &path,
                    std::vector<String> &outFiles,
                    std::vector<String> &outDirs,
                    std::vector<uint32_t> *outSizes = nullptr) {
  File dir = SD.open(path);
  if (!dir || !dir.isDirectory()) return;
  while (true) {
    File entry = dir.openNextFile();
    if (!entry) break;
    String name = entry.name();
    int slash = name.lastIndexOf('/');
    if (slash >= 0) name = name.substring(slash + 1);
    if (entry.isDirectory()) {
      outDirs.push_back(name);
    } else {
      outFiles.push_back(name);
      if (outSizes) outSizes->push_back((uint32_t)entry.size());
    }
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

// [U3 修正] ZIP ストリーミング本体: 事前に Content-Length を計算し、生 client.write()
//          で送る。ZIP は data descriptor 方式 (各ファイル 1 read で済む)
//          無圧縮 (store) 形式、ZIP-32 (4GB 以下) 対応
//
//          ブラウザは Content-Length が確定しているので**進捗バーを表示**できる。
//          chunked encoding を使わないので「0 byte/sec」現象も回避。
//
//          [更に修正] sizes は呼び出し元から渡す。streamZip 内で SD.open() しない。
//                    listDir の段階で entry.size() を取得済みなので追加 SD I/O 不要。
//                    1440 ファイル時: 旧 Pass1 約 43 秒 → 0 秒(瞬時)
static void streamZip(const String &zipName,
                     const std::vector<String> &fullPaths,
                     const std::vector<String> &archiveNames,
                     const std::vector<uint32_t> &sizes) {
  crc32Init();

  // ---- ZIP 全体サイズを計算 (sizes は既知なので SD I/O 不要) ----
  uint32_t totalSize = 0;
  uint32_t cdSize    = 0;
  uint16_t entries   = 0;
  for (size_t i = 0; i < fullPaths.size(); i++) {
    uint16_t namelen = archiveNames[i].length();
    totalSize += 30 + namelen + sizes[i] + 16;   // local hdr + name + data + dd
    cdSize    += 46 + namelen;
    entries++;
  }
  totalSize += cdSize + 22;
  Serial.printf("ZIP: %u entries, total %u bytes\n", entries, totalSize);

  // ---- ヘッダ送信 (Content-Length 確定) ----
  httpSrv.sendHeader("Content-Type", "application/zip");
  httpSrv.sendHeader("Content-Disposition", "attachment; filename=\"" + zipName + "\"");
  httpSrv.sendHeader("Connection", "close");
  httpSrv.setContentLength(totalSize);
  httpSrv.send(200, "application/zip", "");
  WiFiClient client = httpSrv.client();
  // [改善] Nagle's algorithm 無効化: 小〜中サイズの write を即時送信し、ブラウザ
  //       (Vivaldi 等) が短時間で進捗を確認できるようにする
  client.setNoDelay(true);

  // ---- ZIP 本体ストリーミング (data descriptor 方式で 1 read/file) ----
  String centralDir;
  centralDir.reserve(cdSize);
  uint32_t totalOffset = 0;

  uint8_t hdr[30];
  uint8_t dd[16];
  // [改善] バッファを 1KB → 4KB に拡大: SD read 回数を 1/4 に減らし、
  //       WiFi/SD I/O の重なりで全体スループット向上
  uint8_t buf[4096];

  for (size_t i = 0; i < fullPaths.size(); i++) {
    const String &name = archiveNames[i];
    uint16_t namelen = name.length();
    uint32_t size = sizes[i];

    // Local file header (CRC=0, sizes=0, flag bit 3 = data descriptor follows)
    memset(hdr, 0, 30);
    hdr[0]=0x50; hdr[1]=0x4b; hdr[2]=0x03; hdr[3]=0x04;   // signature
    hdr[4]=20;                                              // version
    hdr[6]=0x08;                                            // flag bit 3
    hdr[12]=0x21;                                           // mod date placeholder
    // CRC, sizes は 0 のまま (data descriptor で後出し)
    hdr[26]= namelen & 0xFF; hdr[27]=(namelen>>8)&0xFF;
    client.write(hdr, 30);
    client.write((const uint8_t*)name.c_str(), namelen);

    // ファイル本体: 読みつつ CRC32 計算 + 送信 (1 read のみ)
    File f = SD.open(fullPaths[i]);
    uint32_t crc = 0;
    uint32_t remaining = size;
    while (remaining > 0) {
      uint32_t want = remaining > sizeof(buf) ? sizeof(buf) : remaining;
      int n = f.read(buf, want);
      if (n <= 0) {
        // ファイルが Pass 1 時より短くなった場合: ゼロパディング (Content-Length 維持)
        memset(buf, 0, want);
        n = want;
      }
      crc = crc32Update(crc, buf, n);
      client.write(buf, n);
      remaining -= n;
    }
    f.close();

    // Data descriptor (signature + CRC + compressed size + uncompressed size = 16 byte)
    dd[0]=0x50; dd[1]=0x4b; dd[2]=0x07; dd[3]=0x08;
    dd[4]= crc       & 0xFF; dd[5]=(crc>>8)&0xFF; dd[6]=(crc>>16)&0xFF; dd[7]=(crc>>24)&0xFF;
    dd[8]= size      & 0xFF; dd[9]=(size>>8)&0xFF; dd[10]=(size>>16)&0xFF; dd[11]=(size>>24)&0xFF;
    dd[12]=size      & 0xFF; dd[13]=(size>>8)&0xFF; dd[14]=(size>>16)&0xFF; dd[15]=(size>>24)&0xFF;
    client.write(dd, 16);

    // Central directory entry (確定値で構築)
    uint8_t cd[46];
    memset(cd, 0, 46);
    cd[0]=0x50; cd[1]=0x4b; cd[2]=0x01; cd[3]=0x02;
    cd[4]=20; cd[6]=20;
    cd[8]=0x08;                                       // flag bit 3
    cd[12]=0x21;
    cd[16]= crc       & 0xFF; cd[17]=(crc>>8)&0xFF; cd[18]=(crc>>16)&0xFF; cd[19]=(crc>>24)&0xFF;
    cd[20]= size      & 0xFF; cd[21]=(size>>8)&0xFF; cd[22]=(size>>16)&0xFF; cd[23]=(size>>24)&0xFF;
    cd[24]= size      & 0xFF; cd[25]=(size>>8)&0xFF; cd[26]=(size>>16)&0xFF; cd[27]=(size>>24)&0xFF;
    cd[28]= namelen   & 0xFF; cd[29]=(namelen>>8)&0xFF;
    cd[42]= totalOffset & 0xFF; cd[43]=(totalOffset>>8)&0xFF; cd[44]=(totalOffset>>16)&0xFF; cd[45]=(totalOffset>>24)&0xFF;
    centralDir.concat((const char*)cd, 46);
    centralDir.concat(name);

    totalOffset += 30 + namelen + size + 16;
  }

  // Central directory 送出
  uint32_t cdOffset = totalOffset;
  if (centralDir.length() > 0) {
    client.write((const uint8_t*)centralDir.c_str(), centralDir.length());
  }

  // End of Central Directory record (22 byte)
  uint8_t eocd[22];
  memset(eocd, 0, 22);
  eocd[0]=0x50; eocd[1]=0x4b; eocd[2]=0x05; eocd[3]=0x06;
  eocd[8]=  entries & 0xFF; eocd[9]=(entries>>8)&0xFF;
  eocd[10]= entries & 0xFF; eocd[11]=(entries>>8)&0xFF;
  eocd[12]= cdSize  & 0xFF; eocd[13]=(cdSize>>8)&0xFF; eocd[14]=(cdSize>>16)&0xFF; eocd[15]=(cdSize>>24)&0xFF;
  eocd[16]= cdOffset& 0xFF; eocd[17]=(cdOffset>>8)&0xFF; eocd[18]=(cdOffset>>16)&0xFF; eocd[19]=(cdOffset>>24)&0xFF;
  client.write(eocd, 22);
  client.flush();
  // [改善] 明示的にコネクションを閉じる (Vivaldi が EOF を確実に検出できる)
  client.stop();
}

// [U3] /zip?p=<folder>: 指定フォルダ内の全ファイルを ZIP で DL
static void handleZipFolder() {
  String p = httpSrv.arg("p");
  if (p.length() == 0 || p[0] != '/') { httpSrv.send(400, "text/plain", "bad path"); return; }

  std::vector<String> files, dirs;
  std::vector<uint32_t> sizes;
  listDir(p, files, dirs, &sizes);    // ★ サイズを listDir 段階で収集
  String zipName = p.substring(1) + ".zip";

  std::vector<String> fullPaths, archiveNames;
  for (auto &f : files) {
    fullPaths.push_back(p + "/" + f);
    archiveNames.push_back(f);
  }
  streamZip(zipName, fullPaths, archiveNames, sizes);
}

// [U3] /zipall: SD ルート以下の全ファイルを再帰的に ZIP
static void handleZipAll() {
  std::vector<String> fullPaths, archiveNames;
  std::vector<uint32_t> sizes;

  // ルート直下
  std::vector<String> rootFiles, rootDirs;
  std::vector<uint32_t> rootSizes;
  listDir("/", rootFiles, rootDirs, &rootSizes);
  for (size_t i = 0; i < rootFiles.size(); i++) {
    fullPaths.push_back("/" + rootFiles[i]);
    archiveNames.push_back(rootFiles[i]);
    sizes.push_back(rootSizes[i]);
  }
  // サブフォルダ (1 階層のみ、FIR の /YYYYMMDD/ 想定)
  for (auto &d : rootDirs) {
    std::vector<String> sub, subD;
    std::vector<uint32_t> subSizes;
    listDir("/" + d, sub, subD, &subSizes);
    for (size_t i = 0; i < sub.size(); i++) {
      fullPaths.push_back("/" + d + "/" + sub[i]);
      archiveNames.push_back(d + "/" + sub[i]);
      sizes.push_back(subSizes[i]);
    }
  }
  streamZip("all.zip", fullPaths, archiveNames, sizes);
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
  // [改善] HTTP メソッド指定を省略 → GET/HEAD 双方を受ける。Vivaldi 等が
  //       事前に HEAD でリソース確認するケースに対応。
  httpSrv.on("/",       handleRoot);
  httpSrv.on("/folder", handleFolder);
  httpSrv.on("/dl",     handleDownload);
  httpSrv.on("/zip",    handleZipFolder);
  httpSrv.on("/zipall", handleZipAll);
  httpSrv.begin();

  M5.Lcd.printf("SSID: M5-SEISMO\n");
  M5.Lcd.printf("PASS: m5seismo\n");
  M5.Lcd.println();
  M5.Lcd.printf("Open in browser:\n");
  M5.Lcd.println("http://192.168.4.1");
  M5.Lcd.println();
  M5.Lcd.println("[Download ALL as ZIP]");
  M5.Lcd.println("for full backup.");

  // [U3 追加] Reset ボタン (画面下部)
  //          タップで ESP.restart() → 通常運用に戻る
  const int rbX = 60, rbY = 190, rbW = 200, rbH = 40;
  M5.Lcd.fillRoundRect(rbX,     rbY,     rbW,     rbH,     8, RED);
  M5.Lcd.fillRoundRect(rbX + 4, rbY + 4, rbW - 8, rbH - 8, 8, ORANGE);
  M5.Lcd.setTextFont(4);
  M5.Lcd.setTextColor(BLACK, ORANGE);
  M5.Lcd.setCursor(rbX + 70, rbY + 8);   // 中央寄せ気味
  M5.Lcd.print("Reset");

  while (true) {
    httpSrv.handleClient();
    M5.update();
    auto td = M5.Touch.getDetail();
    if (td.wasPressed()) {
      if (td.x >= rbX && td.x <= rbX + rbW &&
          td.y >= rbY && td.y <= rbY + rbH) {
        // [U3] Reset ボタン押下 → 再起動して通常測定モードへ
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
    {"Reset RTC",     btnY0 + 1 * (btnH + btnGap)},   // [改善] "!" 削除
    {"Manual Set",    btnY0 + 2 * (btnH + btnGap)},
    {"Data Dump",     btnY0 + 3 * (btnH + btnGap)},
  };
  // ボタン描画 (font 4 で読みやすく)
  // [改善] 全ラベルを左揃え (btnX+25)。Wi-Fi Setting が他と同じ X 開始位置になる。
  for (int i = 0; i < 4; i++) {
    M5.Lcd.fillRoundRect(btnX,     btns[i].y,     btnW,     btnH,     btnR, RED);
    M5.Lcd.fillRoundRect(btnX + 4, btns[i].y + 4, btnW - 8, btnH - 8, btnR, ORANGE);
    M5.Lcd.setTextFont(4);
    M5.Lcd.setTextColor(BLACK, ORANGE);
    M5.Lcd.setCursor(btnX + 25, btns[i].y + 8);
    M5.Lcd.print(btns[i].label);
  }
  M5.Lcd.setTextColor(BLACK, WHITE);

  // [U1 改善] 静的なヘッダはループ前に 1 回だけ描画 (ちらつき防止)
  M5.Lcd.setTextFont(4);
  M5.Lcd.setTextColor(BLACK, WHITE);
  M5.Lcd.setCursor(0, 5);
  M5.Lcd.print("Tap a button:");

  // 30 秒カウントダウン (タップ無しでスキップ)
  // [B7] 300 ループ x 100ms = 30 秒
  // [改善] 秒が変わった時だけカウントダウン表示を更新 (ちらつき解消)
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
            case 3: Data_Dump_HTTP(); break;  // 戻ってこない
          }
          dispatched = true;
          break;
        }
      }
    }
    if (dispatched) break;

    int sec = i / 10;
    if (sec != prevSec) {
      // フォント背景色 (WHITE) で上書き → fillRect 不要 → ちらつかない
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
      // [B11] FILE_APPEND も openSDFileSafe で開いて SD 障害時のハング回避
      f = openSDFileSafe(fileName, FILE_APPEND);
    }
  }
}
