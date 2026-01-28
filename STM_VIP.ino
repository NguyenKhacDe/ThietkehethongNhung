#include <Wire.h>
#include <SPI.h>
#include <DHT.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

/* ===== LORA (SPI THUẦN) ===== */
#define LORA_CS   PA4
#define LORA_RST  PA1

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

#define MODE_LONG_RANGE 0x80
#define MODE_SLEEP      0x00
#define MODE_STDBY      0x01
#define MODE_TX         0x03
#define MODE_RX_CONT    0x05

#define IRQ_RX_DONE 0x40
#define IRQ_TX_DONE 0x08
#define IRQ_CRC_ERR 0x20

static inline void csLow(){ digitalWrite(LORA_CS, LOW); }
static inline void csHigh(){ digitalWrite(LORA_CS, HIGH); }

uint8_t readReg(uint8_t addr){
  csLow(); SPI.transfer(addr & 0x7F);
  uint8_t v = SPI.transfer(0);
  csHigh(); return v;
}
void writeReg(uint8_t addr, uint8_t val){
  csLow(); SPI.transfer(addr | 0x80); SPI.transfer(val);
  csHigh();
}
void resetRadio(){
  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, LOW); delay(10);
  digitalWrite(LORA_RST, HIGH); delay(10);
}
void setFreq(long hz){
  uint64_t frf = ((uint64_t)hz << 19) / 32000000ULL;
  writeReg(REG_FRF_MSB, frf >> 16);
  writeReg(REG_FRF_MID, frf >> 8);
  writeReg(REG_FRF_LSB, frf);
}
void radioRX(){
  writeReg(REG_IRQ_FLAGS, 0xFF);
  writeReg(REG_OP_MODE, MODE_LONG_RANGE | MODE_RX_CONT);
}
void radioInit(){
  writeReg(REG_OP_MODE, MODE_LONG_RANGE | MODE_SLEEP); delay(5);
  writeReg(REG_OP_MODE, MODE_LONG_RANGE | MODE_STDBY); delay(5);
  setFreq(433000000);
  writeReg(REG_MODEM_CONFIG_1, 0x72);      // BW125 + CR4/5
  writeReg(REG_MODEM_CONFIG_2, 0x74);      // SF7 + CRC
  writeReg(REG_MODEM_CONFIG_3, 0x04);
  writeReg(REG_PREAMBLE_MSB, 0);
  writeReg(REG_PREAMBLE_LSB, 8);
  writeReg(REG_SYNC_WORD, 0x12);
  writeReg(REG_PA_CONFIG, 0x8C);           // ~14dBm
  writeReg(REG_FIFO_RX_BASE_ADDR, 0);
  writeReg(REG_FIFO_ADDR_PTR, 0);
  radioRX();
}
void sendPacket(String s){
  writeReg(REG_OP_MODE, MODE_LONG_RANGE | MODE_STDBY);
  writeReg(REG_IRQ_FLAGS, 0xFF);
  writeReg(REG_FIFO_TX_BASE_ADDR, 0);
  writeReg(REG_FIFO_ADDR_PTR, 0);
  csLow(); SPI.transfer(REG_FIFO | 0x80);
  for(char c: s) SPI.transfer(c);
  csHigh();
  writeReg(REG_PAYLOAD_LENGTH, s.length());
  writeReg(REG_OP_MODE, MODE_LONG_RANGE | MODE_TX);
  while(!(readReg(REG_IRQ_FLAGS) & IRQ_TX_DONE));
  writeReg(REG_IRQ_FLAGS, 0xFF);
  //radioRX();
}

/* ===== OLED ===== */
#define I2C_ADDRESS 0x3C
SSD1306AsciiWire oled;

/* ===== DHT ===== */
#define DHTPIN PB1
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

/* ===== RELAY ===== */
#define Fan  PB12
#define Lamp PB13

int TEMP_MAX=35, TEMP_MIN=25;
int Temp=0, Hum=0;
String strQuat="OFF", strDen="OFF";
unsigned long lastOLED=0;

/* ===== UPDATE RELAY ===== */
void updateRelay(){
  if(Temp > TEMP_MAX){
    digitalWrite(Fan, HIGH);
    digitalWrite(Lamp, LOW);
  }
  else if(Temp < TEMP_MIN){
    digitalWrite(Fan, LOW);
    digitalWrite(Lamp, HIGH);
  }
  else{
    digitalWrite(Fan, LOW);
    digitalWrite(Lamp, LOW);
  }
}

void setup(){
  Serial1.begin(115200);
  pinMode(Fan,OUTPUT); pinMode(Lamp,OUTPUT);
  Wire.begin();
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(Adafruit5x7);
  oled.println("STM32 READY");
  dht.begin();
  SPI.begin();
  pinMode(LORA_CS, OUTPUT); csHigh();
  resetRadio(); radioInit();
}

void loop(){
  uint8_t irq = readReg(REG_IRQ_FLAGS);

  if(irq & IRQ_RX_DONE){
    uint8_t n = readReg(REG_RX_NB_BYTES);
    uint8_t addr = readReg(REG_FIFO_RX_CURRENT);
    writeReg(REG_FIFO_ADDR_PTR, addr);

    char buf[64]={0};
    csLow(); SPI.transfer(REG_FIFO & 0x7F);
    for(int i=0;i<n;i++) buf[i]=SPI.transfer(0);
    csHigh();

    writeReg(REG_IRQ_FLAGS,0xFF); // clear sau khi đọc

    String rx = String(buf);
    Serial1.println("RX: "+rx);

    if(rx.startsWith("REQ")){
      int colon = rx.indexOf(':');
      int comma = rx.indexOf(',');

      if(colon > 0 && comma > colon){
        TEMP_MAX = rx.substring(colon + 1, comma).toInt();
        TEMP_MIN = rx.substring(comma + 1).toInt();

        Serial1.println("Nhan TMAX/TMIN: "
          + String(TEMP_MAX) + "/" + String(TEMP_MIN));
      }

      Temp = dht.readTemperature();
      Hum  = dht.readHumidity();

      updateRelay();

      sendPacket("S:T=" + String(Temp) + ",H=" + String(Hum));

      Serial1.println("Da gui T/H"+String(Temp)+String(Hum));
    }
     radioRX(); // ✅ RX lại tại đây

  }

  if(millis()-lastOLED>2000){
    lastOLED=millis();
    oled.clear();
    oled.println("--- GIAM SAT ---");
    oled.print("T: "); oled.println(Temp);
    oled.print("H: "); oled.println(Hum);
    oled.print("MAX/MIN: ");
    oled.print(TEMP_MAX); oled.print("/");
    oled.println(TEMP_MIN);
  }
}
