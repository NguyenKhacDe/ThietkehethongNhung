#include <SPI.h>
#include <WiFi.h>
#include <FirebaseESP32.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>

/* ================= LORA PIN ================= */
#define PIN_SS   27
#define PIN_RST  32


/* ================= SX127x REG ================= */
#define REG_FIFO               0x00
#define REG_OP_MODE            0x01
#define REG_FRF_MSB            0x06
#define REG_FRF_MID            0x07
#define REG_FRF_LSB            0x08
#define REG_PA_CONFIG          0x09
#define REG_FIFO_ADDR_PTR      0x0D
#define REG_FIFO_TX_BASE_ADDR  0x0E
#define REG_FIFO_RX_BASE_ADDR  0x0F
#define REG_FIFO_RX_CURRENT    0x10
#define REG_IRQ_FLAGS          0x12
#define REG_RX_NB_BYTES        0x13
#define REG_PKT_RSSI_VALUE     0x1A
#define REG_MODEM_CONFIG_1     0x1D
#define REG_MODEM_CONFIG_2     0x1E
#define REG_PREAMBLE_MSB       0x20
#define REG_PREAMBLE_LSB       0x21
#define REG_PAYLOAD_LENGTH     0x22
#define REG_MODEM_CONFIG_3     0x26
#define REG_SYNC_WORD          0x39
#define REG_VERSION            0x42

#define MODE_LONG_RANGE 0x80
#define MODE_SLEEP      0x00
#define MODE_STDBY      0x01
#define MODE_TX         0x03
#define MODE_RX_CONT    0x05

#define IRQ_RX_DONE 0x40
#define IRQ_TX_DONE 0x08
#define IRQ_CRC_ERR 0x20

/* ================= WIFI + FIREBASE ================= */
#define WIFI_SSID "UET-Wifi-Office-Free 2.4Ghz"
#define WIFI_PASSWORD ""
#define API_KEY "AIzaSyA5tgM4SQfplDgXHI5BGga4XWCzDyPegYU"
#define DATABASE_URL "my-project-k32-default-rtdb.asia-southeast1.firebasedatabase.app"

FirebaseData fbdo_send, fbdo_stream;
FirebaseAuth auth;
FirebaseConfig config;

/* ================= BIẾN ================= */
int temp = 0, hum = 0;
int tempMax = 35, tempMin = 25;

unsigned long lastPoll   = 0;
unsigned long lastUpload = 0;

/* ================= SPI LOW LEVEL ================= */
static inline void csLow()  { digitalWrite(PIN_SS, LOW); }
static inline void csHigh() { digitalWrite(PIN_SS, HIGH); }

uint8_t readReg(uint8_t addr) {
  csLow();
  SPI.transfer(addr & 0x7F);
  uint8_t v = SPI.transfer(0x00);
  csHigh();
  return v;
}

void writeReg(uint8_t addr, uint8_t val) {
  csLow();
  SPI.transfer(addr | 0x80);
  SPI.transfer(val);
  csHigh();
}

void resetRadio() {
  pinMode(PIN_RST, OUTPUT);
  digitalWrite(PIN_RST, LOW);  delay(10);
  digitalWrite(PIN_RST, HIGH); delay(10);
}

void setFrequency(long hz) {
  uint64_t frf = ((uint64_t)hz << 19) / 32000000ULL;
  writeReg(REG_FRF_MSB, frf >> 16);
  writeReg(REG_FRF_MID, frf >> 8);
  writeReg(REG_FRF_LSB, frf);
}

/* ================= LORA CORE ================= */
void radioEnterRx() {
  writeReg(REG_IRQ_FLAGS, 0xFF);
  writeReg(REG_OP_MODE, MODE_LONG_RANGE | MODE_RX_CONT);
}

void radioInit() {
  writeReg(REG_OP_MODE, MODE_LONG_RANGE | MODE_SLEEP); delay(5);
  writeReg(REG_OP_MODE, MODE_LONG_RANGE | MODE_STDBY); delay(5);

  setFrequency(433000000);

  writeReg(REG_MODEM_CONFIG_1, 0x72);   // BW125 + CR4/5
  writeReg(REG_MODEM_CONFIG_2, 0x74);   // SF7 + CRC ON
  writeReg(REG_MODEM_CONFIG_3, 0x04);   // AGC auto

  writeReg(REG_PREAMBLE_MSB, 0x00);
  writeReg(REG_PREAMBLE_LSB, 0x08);
  writeReg(REG_SYNC_WORD, 0x12);

  writeReg(REG_PA_CONFIG, 0x8C);        // ~14 dBm

  writeReg(REG_FIFO_RX_BASE_ADDR, 0x00);
  writeReg(REG_FIFO_ADDR_PTR, 0x00);

  radioEnterRx();
}

void sendPacket(String s) {
  int n = s.length();

  writeReg(REG_OP_MODE, MODE_LONG_RANGE | MODE_STDBY);
  writeReg(REG_IRQ_FLAGS, 0xFF);

  writeReg(REG_FIFO_TX_BASE_ADDR, 0x00);
  writeReg(REG_FIFO_ADDR_PTR, 0x00);

  csLow();
  SPI.transfer(REG_FIFO | 0x80);
  for (int i = 0; i < n; i++) SPI.transfer(s[i]);
  csHigh();

  writeReg(REG_PAYLOAD_LENGTH, n);
  writeReg(REG_OP_MODE, MODE_LONG_RANGE | MODE_TX);

  while (!(readReg(REG_IRQ_FLAGS) & IRQ_TX_DONE));
  writeReg(REG_IRQ_FLAGS, 0xFF);

  radioEnterRx();
}

/* ================= RX ================= */
bool receivePacket(String &out) {
  uint8_t irq = readReg(REG_IRQ_FLAGS);
  if (!(irq & IRQ_RX_DONE)) return false;

  if (irq & IRQ_CRC_ERR) {
    writeReg(REG_IRQ_FLAGS, 0xFF);
    return false;
  }

  uint8_t n = readReg(REG_RX_NB_BYTES);
  uint8_t addr = readReg(REG_FIFO_RX_CURRENT);

  writeReg(REG_FIFO_ADDR_PTR, addr);

  char buf[128] = {0};
  csLow();
  SPI.transfer(REG_FIFO & 0x7F);
  for (int i = 0; i < n && i < 127; i++)
    buf[i] = SPI.transfer(0);
  csHigh();

  writeReg(REG_IRQ_FLAGS, 0xFF);
  out = String(buf);
  return true;
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(PIN_SS, OUTPUT);
  csHigh();

  SPI.begin(18, 19, 23);
  resetRadio();

  Serial.print("SX127x VERSION: 0x");
  Serial.println(readReg(REG_VERSION), HEX);

  radioInit();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) delay(300);
  Serial.println("WiFi OK");

  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  Firebase.signUp(&config, &auth, "", "");
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  Firebase.beginStream(fbdo_stream, "/");
}

/* ================= LOOP ================= */
void loop() {
  /* ===== 1. POLL STM32, Khóa REG khi có CONFIG ===== */
  if (millis() - lastPoll > 5000) {
    lastPoll = millis();
    String req = "REQ:" + String(tempMax) + "," + String(tempMin);
    sendPacket(req);
    Serial.println("-> TX: "+req);
  }

  /* ===== 2. RX LORA ===== */
  String rx;
  if (receivePacket(rx)) {
    Serial.println("RX: " + rx);

    if (rx.startsWith("S:")) {
      int t = rx.indexOf("T=");
      int h = rx.indexOf(",H=");
      if (t >= 0 && h >= 0) {
        temp = rx.substring(t + 2, h).toInt();
        hum  = rx.substring(h + 3).toInt();
      }
    }
  }

  /* ===== 3. FIREBASE STREAM ===== */
  if (Firebase.readStream(fbdo_stream)) {
    if (fbdo_stream.streamAvailable()) {

      FirebaseJson &json = fbdo_stream.jsonObject();
      FirebaseJsonData jd;

      if (json.get(jd, "temp_max")) {
        if (tempMax != jd.intValue) {
          tempMax = jd.intValue;
        
        }
      }

      if (json.get(jd, "temp_min")) {
        if (tempMin != jd.intValue) {
          tempMin = jd.intValue;
        
        }
      }
    }
  }

  /* ===== 4. UPLOAD SENSOR ===== */
  if (millis() - lastUpload > 5000) {
    lastUpload = millis();

    FirebaseJson js;
    js.set("temperature", temp);
    js.set("humidity", hum);
    Firebase.setJSON(fbdo_send, "/sensor", js);
  }
}
