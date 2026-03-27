#include <Arduino.h>
#include <esp_sleep.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "Audio.h"
#include <NimBLEDevice.h>
#include <WiFi.h>
#include "audio_data.h"
#include <Adafruit_MLX90614.h>

//Pin
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

Audio audio;
Adafruit_MPU6050 mpu;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// --- Variables ---
float currentBodyTemp = 0;

float pitchOffset = 0;
unsigned long postureStartTime = 0;
unsigned long lastSync = 0
bool isSlouching = false;
unsigned long lastMoveTime = 0;
float lastPitch = 0;
const float moveThreshold = 5.0;

bool isTalking = false;
unsigned long wifiStartTime = 0;
const unsigned long timeoutLimit = 120000;

BLECharacteristic* pTxCharacteristic;
bool deviceConnected = false;

enum SleepReason {
    SLEEP_AUTO,   // กางขาอยู่ แต่ไม่ขยับ 5 นาที
    SLEEP_FOLDED  // พับขาแว่น
};

void setupSensors() {
    if (!mpu.begin()) Serial.println("MPU6050 Fail!");
    if (!mlx.begin()) Serial.println("MLX90614 Fail!");
    
    // ตั้งค่า MPU ให้แม่นยำขึ้น
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
}

void sendDataToApp() {
    if (deviceConnected) {
        // อ่านแบตเตอรี่
        int rawADC = analogRead(BATT_PIN);
        float vBat = (rawADC / 4095.0) * 3.3 * 2.0;
        int pBat = map(constrain(vBat*100, 330, 415), 330, 415, 0, 100);

        // รวมร่าง String: "T:36.5,B:85" (ประหยัดพื้นที่ส่ง)
        String payload = "T:" + String(currentBodyTemp, 1) + ",B:" + String(pBat);
        
        pTxCharacteristic->setValue(payload.c_str());
        pTxCharacteristic->notify();
        
        Serial.println(">>> Sent: " + payload);
    }
}

void updateTemperature() {
    currentBodyTemp = mlx.readObjectTempC();

}

void calibrateMPU() {
    sensors_event_t a, g, temp;
    float totalAngle = 0;
    for(int i=0; i<10; i++) {
        mpu.getEvent(&a, &g, &temp);
        totalAngle += atan2(a.acceleration.y, a.acceleration.x) * 180 / PI;
        delay(10);
    }
    pitchOffset = totalAngle / 10;
    Serial.println("Calibration Done!");
}

// --- BLE Callbacks ---
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("App Connected!");
    }
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("App Disconnected!");
        pServer->getAdvertising()->start();
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        if (rxValue.length() > 0) {
            String cmd = String(rxValue.c_str());
            cmd.trim();
            if (cmd == "CAL") calibrateMPU();
        }
    }
};

// --- Functions ---

void setupBLE() {
    BLEDevice::init("AURA_1");
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
    pTxCharacteristic = pService->createCharacteristic("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", NIMBLE_PROPERTY::NOTIFY);
    BLECharacteristic *pRxCharacteristic = pService->createCharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", NIMBLE_PROPERTY::WRITE);
    pRxCharacteristic->setCallbacks(new MyCallbacks());
    pService->start();
    pServer->getAdvertising()->start();
}

float getPitch() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float angle = atan2(a.acceleration.y, a.acceleration.x) * 180 / PI;
    return angle - pitchOffset;
}

// ระบบบันทึกเสียง/WiFi (เพิ่มใหม่ตามที่คุยกัน)
void checkTouchLogic() {
    int touchVal = touchRead(TOUCH_PIN);
    if (touchVal < 30 && !isTalking) {
        isTalking = true;
        wifiStartTime = millis();
        lastMoveTime = millis();
        setCpuFrequencyMhz(240); 
        digitalWrite(I2S_SD, HIGH);
        delay(50);
        // ใช้คำสั่งที่พิสูจน์แล้วว่าผ่าน!
        audio.connecttohost((const char*)beep_wav);
        WiFi.begin("จากเปียง", "Kompisit");
    }
    
    if (isTalking && (touchRead(TOUCH_PIN) > 50 || (millis() - wifiStartTime > timeoutLimit))) {
        isTalking = false;
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        digitalWrite(I2S_SD, LOW);
        setCpuFrequencyMhz(80);
    }
}

void checkPostureLogic() {
    float currentPitch = getPitch();
    if (currentPitch > 30.0) {
        if (!isSlouching) {
            postureStartTime = millis();
            isSlouching = true;
        } else if (millis() - postureStartTime > 60000) {
            if (!audio.isRunning()) {
                digitalWrite(I2S_SD, HIGH);
                audio.connecttohost((const char*)beep_wav); // หรือไฟล์เสียงเตือนอื่น
            }
        }
    } else {
        isSlouching = false;
    }
}

void enterLightSleep(SleepReason reason) {

    // ═══ STEP 1: ปิดทุกอย่างก่อนหลับ ═══
    Serial.println(reason == SLEEP_FOLDED ? "Folded -> Sleep" : "Auto -> Sleep");
    Serial.flush();

    audio.stopSong();
    digitalWrite(I2S_SD, LOW);

    if (isTalking) {          // ถ้า WiFi กำลังเปิดอยู่ ปิดด้วย
        isTalking = false;
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
    }
    setCpuFrequencyMhz(80);   // ลด CPU ก่อนหลับประหยัดไฟ

    // ═══ STEP 2: วน Sleep Loop ═══
    // เงื่อนไขออกจาก loop ต่างกันตาม reason
    bool shouldWakeUp = false;

    while (!shouldWakeUp) {

    gpio_wakeup_enable((gpio_num_t)HALL_SENSOR, GPIO_INTR_LOW_LEVEL);
    gpio_wakeup_enable((gpio_num_t)USB_DETECT_PIN, GPIO_INTR_HIGH_LEVEL);
    esp_sleep_enable_timer_wakeup(30 * 1000000ULL);
    esp_sleep_enable_gpio_wakeup();

    esp_light_sleep_start(); // 😴 หลับ

    // ตื่นมา → เช็คสาเหตุ
    bool hallOpen = (digitalRead(HALL_SENSOR)    == LOW);
    bool usbIn    = (digitalRead(USB_DETECT_PIN) == HIGH);

    // [กรณี A] เปิดขาแว่น → ตื่นจริง ออกไปใช้งาน
    if (hallOpen) {
        Serial.println("Woke: Glasses opened!");
        shouldWakeUp = true;
        break;
    }

    // [กรณี B] Timer ครบ 30 วิ → ส่งแบตแล้วหลับต่อ
    // [กรณี B+] เสียบ USB → ส่งแบตเหมือนกัน แล้วหลับต่อ
    // (usbIn ใช้แค่แยก log ให้รู้ว่าตื่นเพราะอะไร)
    if (usbIn) {
        Serial.println("Woke: USB plugged in, updating battery then sleep again");
    } else {
        Serial.println("Woke: Timer 30s, updating battery then sleep again");
    }

    updateTemperature();
    sendDataToApp(); // ส่งแบต+temp ไป App ผ่าน BLE

    // วนกลับหลับต่อทั้งสองกรณี ✅
}
    // ═══ STEP 3: ฟื้นระบบหลังตื่นจริง ═══
    lastMoveTime = millis();
    setCpuFrequencyMhz(240); // คืน CPU speed
    Serial.println("System fully awake!");
}

void setup() {
    Serial.begin(115200);

    Wire.begin(SDA_PIN, SCL_PIN);
    
    pinMode(I2S_SD, OUTPUT);
    digitalWrite(I2S_SD, LOW);
    audio.setPinout(I2S_BCLK, I2S_LRCK, I2S_DOUT);

    setupSensors();
    setupBLE();

    pinMode(HALL_SENSOR, INPUT_PULLUP);
    pinMode(USB_DETECT_PIN, INPUT);
    analogReadResolution(12);
}

void loop() {
    // พับขาแว่น → ส่ง reason ไปด้วย
    if (digitalRead(HALL_SENSOR) == HIGH) {
        delay(50);  // debounce กันสัญญาณกระเพื่อม
        if (digitalRead(HALL_SENSOR) == HIGH) {     // เช็คซ้ำให้แน่ใจ
            enterLightSleep(SLEEP_FOLDED);
            return;
        }
    }

    audio.loop();
    checkTouchLogic();
    checkPostureLogic();

    if (deviceConnected && (millis() - lastSync > 5000)) {
        updateTemperature();
        sendDataToApp();
        lastSync = millis();
    }

    checkAutoSleep();
}

void checkAutoSleep() {
    if (millis() - lastMoveTime > 300000) {  // 5 นาที
        enterLightSleep(SLEEP_AUTO);          // ← ส่ง reason ไปด้วย
    }
}





