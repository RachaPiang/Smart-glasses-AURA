#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MLX90614.h>
#include <Audio.h>
#include <NimBLEDevice.h>
#include <WiFi.h>
#include <esp_sleep.h>
#include <driver/i2s.h>
#include <Preferences.h>
#include <WiFiMulti.h>

// ===== PIN DEFINITIONS =====
#define HALL_SENSOR     D0
#define I2S_BCLK        D1
#define I2S_LRCK        D2
#define I2S_DOUT        D3
#define SDA_PIN         D4
#define SCL_PIN         D5
#define I2S_MIC         D6
#define TOUCH_PIN       D7
#define I2S_SD          D8
#define USB_DETECT_PIN  D9
#define BATT_PIN        D10

#define LED_BUILTIN     21
#define GLASSES_OPEN    0
#define GLASSES_FOLDED  1

// ===== OBJECTS =====
Audio             audio;
Adafruit_MPU6050  mpu;
Adafruit_MLX90614 mlx;

// ===== BLE =====
BLECharacteristic* pTxCharacteristic = nullptr;
bool deviceConnected = false;

// ===== SENSORS =====
bool  mpuOK = false;
bool  mlxOK = false;
const float TEMP_OFFSET = 2.0;

// ===== POSTURE =====
float pitchOffset      = 0.0;
bool  isSlouching      = false;
unsigned long postureStartTime = 0;

// ===== STEP COUNTER =====
RTC_DATA_ATTR int stepCount = 0;
bool  peakDetected = false;
unsigned long lastStepTime = 0;

// ===== TOUCH / WIFI / AI =====
bool     isTalking   = false;
int16_t* audioBuffer = nullptr;
size_t   audioLength = 0;
String   wifiSSID    = "";
String   wifiPASS    = "";

// ===== BATTERY =====
const float DIVIDER_RATIO = 2.0;
const float BATT_MAX      = 4.15;
const float BATT_MIN      = 3.30;
RTC_DATA_ATTR int lastBattPercent = 0;

// ===== TIMING =====
unsigned long lastMoveTime = 0;
unsigned long lastSync     = 0;
unsigned long lastPrintTime = 0;

// ===== SLEEP =====
enum SleepReason { SLEEP_FOLDED, SLEEP_AUTO };

// ===== WiFi ======
Preferences preferences;
WiFiMulti wifiMulti;

// ========================================
// SENSORS
// ========================================
void setupSensors() {
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);

    if (mpu.begin()) {
        mpuOK = true;
        mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
        mpu.setGyroRange(MPU6050_RANGE_250_DEG);
        Serial.println("MPU6050 OK");
    } else {
        Serial.println("MPU6050 FAIL");
    }

    if (mlx.begin()) {
        mlxOK = true;
        Serial.println("MLX90614 OK");
        delay(500);
    } else {
        Serial.println("MLX90614 FAIL");
    }
}

float getBodyTemp() {
    if (!mlxOK) return -1;
    float obj = mlx.readObjectTempC();
    float amb = mlx.readAmbientTempC();
    if (isnan(obj) || isnan(amb)) return -1;
    return obj + TEMP_OFFSET;
}

float getGlassTemp() {
    if (!mlxOK) return -1;
    return mlx.readAmbientTempC();
}

// ========================================
// BATTERY
// ========================================
int getBatteryPercent(bool &isCharging) {
    isCharging = (digitalRead(USB_DETECT_PIN) == HIGH);

    int sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += analogRead(BATT_PIN);
        delay(5);
    }
    float raw  = sum / 10.0;
    float vPin = (raw / 4095.0) * 3.3;
    float vBat = vPin * DIVIDER_RATIO;
    if (isCharging) vBat -= 0.15;

    float percent = (constrain(vBat, BATT_MIN, BATT_MAX) - BATT_MIN)
                    / (BATT_MAX - BATT_MIN) * 100.0;
    lastBattPercent = (int)percent;
    return lastBattPercent;
}

// ========================================
// POWER DOWN MODULES
// ========================================
void powerDownModules() {
    if (mpuOK) {
        Wire.beginTransmission(0x68);
        Wire.write(0x6B);
        Wire.write(0x40);
        Wire.endTransmission();
    }
    Wire.end();
    digitalWrite(I2S_SD, LOW);
    digitalWrite(LED_BUILTIN, HIGH);
}

// ========================================
// SLEEP
// ========================================
void goToDeepSleep() {
    powerDownModules(); // ✅ ปิด Module ก่อนหลับ

    esp_sleep_enable_ext0_wakeup((gpio_num_t)HALL_SENSOR, GLASSES_OPEN);
    esp_sleep_enable_timer_wakeup(30 * 1000000ULL);

    Serial.println("Deep Sleep...");
    Serial.flush();
    delay(100);

    esp_deep_sleep_start();
}

void enterSleep(SleepReason reason) {
    Serial.println(reason == SLEEP_FOLDED ? "พับขา → รอ 5 วิ..." : "ไม่ขยับ → Auto Sleep");
    Serial.flush();

    if (reason == SLEEP_AUTO && digitalRead(HALL_SENSOR) == GLASSES_OPEN) {
        Serial.println("Auto Sleep ยกเลิก: ขาแว่นเปิดอยู่");
        lastMoveTime = millis();
        return;
    }

    bool isCharging;
    lastBattPercent = getBatteryPercent(isCharging);
    Serial.print("แบตก่อนหลับ: "); Serial.print(lastBattPercent); Serial.println("%");

    if (reason == SLEEP_FOLDED) {
        for (int i = 5; i > 0; i--) {
            delay(1000);
            if (digitalRead(HALL_SENSOR) == GLASSES_OPEN) {
                Serial.println("ยกเลิก!");
                return;
            }
            Serial.print("Sleep ใน "); Serial.print(i); Serial.println(" วิ...");
        }
    }

    // กะพริบ 3 ครั้ง = กำลังจะหลับ
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_BUILTIN, LOW);  delay(200);
        digitalWrite(LED_BUILTIN, HIGH); delay(200);
    }

    goToDeepSleep();
}

void checkAutoSleep() {
    if (millis() - lastMoveTime > 300000) {
        enterSleep(SLEEP_AUTO);
    }
}

// ========================================
// NVS & WIFI SETUP
// ========================================
void loadWiFiConfig() {
    preferences.begin("smartglass", true); // เปิดโหมด Read-only
    
    Serial.println("[NVS] โหลดข้อมูล WiFi...");
    
    // ดึงค่า WiFi ทั้ง 3 Slot แล้วโยนเข้า WiFiMulti
    for (int i = 1; i <= 3; i++) {
        String keySsid = "ssid" + String(i);
        String keyPass = "pass" + String(i);
        
        String ssid = preferences.getString(keySsid.c_str(), "");
        String pass = preferences.getString(keyPass.c_str(), "");
        
        if (ssid != "") {
            wifiMulti.addAP(ssid.c_str(), pass.c_str()); // เพิ่มเข้าระบบ WiFiMulti
            Serial.print("Slot "); Serial.print(i); 
            Serial.print(" : "); Serial.println(ssid);
        }
    }
    preferences.end();
}

void saveWiFiConfig(int slot, String ssid, String pass) {
    if (slot < 1 || slot > 3) return; // กันพัง บังคับแค่ Slot 1-3

    preferences.begin("smartglass", false); // เปิดโหมด Read/Write
    
    String keySsid = "ssid" + String(slot);
    String keyPass = "pass" + String(slot);
    
    preferences.putString(keySsid.c_str(), ssid);
    preferences.putString(keyPass.c_str(), pass);
    
    preferences.end();
    
    Serial.print("[NVS] บันทึก WiFi Slot "); 
    Serial.print(slot); Serial.println(" สำเร็จ! (มีผลตอนเปิดเครื่องใหม่)");
}

void connectToWiFi() {
    Serial.println("[WIFI] กำลังเชื่อมต่อ...");
    
    // wifiMulti.run() จะสแกนและพยายามต่อตัวที่เจอและสัญญาณดีสุด
    if (wifiMulti.run() == WL_CONNECTED) {
        Serial.print("[WIFI] เชื่อมต่อสำเร็จ! เครือข่าย: ");
        Serial.println(WiFi.SSID());
    } else {
        Serial.println("[WIFI] เชื่อมต่อไม่สำเร็จ (ไม่เจอ WiFi ที่บันทึกไว้เลย)");
    }
}

// ========================================
// SETUP
// ========================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(HALL_SENSOR,    INPUT_PULLUP);
    pinMode(USB_DETECT_PIN, INPUT);
    pinMode(BATT_PIN,       INPUT);
    pinMode(TOUCH_PIN,      INPUT);
    pinMode(I2S_SD,         OUTPUT);
    digitalWrite(I2S_SD,    LOW);
    pinMode(LED_BUILTIN,    OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    loadWiFiConfig();

    saveWiFiConfig(1, "pulifting", "07537873");
    saveWiFiConfig(2, "จากเปียง", "Kompisit");

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

    if (cause == ESP_SLEEP_WAKEUP_EXT0) {
        // เปิดขาแว่น → ใช้งานปกติ
        Serial.println("ตื่น: เปิดขาแว่น!");
        for (int i = 0; i < 5; i++) {
            digitalWrite(LED_BUILTIN, LOW);  delay(100);
            digitalWrite(LED_BUILTIN, HIGH); delay(100);
        }
        digitalWrite(LED_BUILTIN, LOW);

    } else if (cause == ESP_SLEEP_WAKEUP_TIMER) {
        // Timer 30 วิ → อัพเดทแบตแล้วหลับต่อทันที
        Serial.println("Timer: อัพเดทแบต...");
        Wire.begin(SDA_PIN, SCL_PIN);
        bool isCharging;
        lastBattPercent = getBatteryPercent(isCharging);
        Serial.print("แบต: "); Serial.print(lastBattPercent); Serial.println("%");
        Serial.flush();
        goToDeepSleep();

    } else {
        // เปิดเครื่องครั้งแรก
        Serial.println("เปิดเครื่องครั้งแรก");
        digitalWrite(LED_BUILTIN, LOW);

    }

    setupSensors();
    lastMoveTime = millis();
    Serial.println("พร้อมใช้งาน!");
}

// ========================================
// LOOP
// ========================================
void loop() {

    if (digitalRead(HALL_SENSOR) == GLASSES_FOLDED) {
        delay(50); // Debounce กันปุ่มเบิ้ล
        if (digitalRead(HALL_SENSOR) == GLASSES_FOLDED) {
            enterSleep(SLEEP_FOLDED);
            return;
        }
    }

    checkAutoSleep();
    sensors_event_t a, g, temp;
    if (mpuOK) {
        mpu.getEvent(&a, &g, &temp);
        
        // 📌 อนาคต: โค้ด Block 10 (ก้มหน้า) และ Block 11 (นับก้าว) 
        // จะถูกเรียกใช้งานตรงนี้ เพื่อไม่ให้พลาดทุกการเคลื่อนไหว
    }

    if (millis() - lastPrintTime >= 1000) {
        lastPrintTime = millis(); // อัพเดทเวลาล่าสุด

        // Print ค่า MPU6050 ที่อ่านได้ล่าสุดจากเลนด่วน
        if (mpuOK) {
            Serial.print("[MPU] Accel X:"); Serial.print(a.acceleration.x, 2);
            Serial.print(" Y:");            Serial.print(a.acceleration.y, 2);
            Serial.print(" Z:");            Serial.print(a.acceleration.z, 2);
            Serial.print(" | Gyro X:");     Serial.print(g.gyro.x, 2);
            Serial.print(" Y:");            Serial.print(g.gyro.y, 2);
            Serial.print(" Z:");            Serial.println(g.gyro.z, 2);
        }

        // อ่านและ Print เซ็นเซอร์ที่เปลี่ยนแปลงช้า (อุณหภูมิ)
        if (mlxOK) {
            float bodyTemp  = getBodyTemp();
            float glassTemp = getGlassTemp();
            if (bodyTemp == -1) {
                Serial.println("[MLX] อ่านค่าไม่ได้");
            } else {
                Serial.print("[MLX] Body:");  Serial.print(bodyTemp, 1);
                Serial.print("°C  Glass:");  Serial.print(glassTemp, 1);
                Serial.println("°C");
            }
        }

        // อ่านและ Print แบตเตอรี่
        bool isCharging;
        int batt = getBatteryPercent(isCharging);
        Serial.print("[BAT] ");
        Serial.print(batt);
        Serial.print("%");
        Serial.println(isCharging ? " ⚡ ชาร์จอยู่" : "");

        // โชว์สถานะขาแว่น
        Serial.print("[HALL] ");
        Serial.println(digitalRead(HALL_SENSOR) == GLASSES_OPEN ? "เปิดอยู่" : "พับอยู่");

        Serial.println("─────────────────");
    }
}