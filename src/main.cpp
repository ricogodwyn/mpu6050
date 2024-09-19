#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "WiFi.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;
#define INTERRUPT_PIN 2
#define LED_PIN 13
bool blinkState = false;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float euler[3];
float ypr[3];

const char *ssid = "ficofichky";
const char *password = "ficofichkymanuetha_123";
WiFiServer server(80);

volatile bool mpuInterrupt = false;
void dmpDataReady()
{
    mpuInterrupt = true;
}

void vMPU6050Task(void *pvParameters)
{
    for (;;)
    {
        if (!dmpReady)
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
        {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print(q.w);
            Serial.print(",");
            Serial.print(q.x);
            Serial.print(",");
            Serial.print(q.y);
            Serial.println(q.z);

            blinkState = !blinkState;
            digitalWrite(LED_PIN, blinkState);

            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
    }
}

void vWiFiServerTask(void *pvParameters)
{
    for (;;)
    {
        WiFiClient client = server.available();
        if (client)
        {
            Serial.println("Client connected");
            while (client.connected())
            {
                String message = String(q.w) + "," + String(q.x) + "," + String(q.y) + "," + String(q.z);
                client.println(message);
                vTaskDelay(500 / portTICK_PERIOD_MS);
            }
            client.stop();
            Serial.println("Client disconnected");
        }
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}

void setup()
{
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin(21, 22);
    Wire.setClock(100000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    Serial.begin(9600);
    Serial.println("Begin!!");
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }

    Serial.println("Connected to WiFi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    server.begin();
    Serial.println("Bluetooth started");

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if (devStatus == 0)
    {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    pinMode(LED_PIN, OUTPUT);

    // Create FreeRTOS tasks
    xTaskCreatePinnedToCore(vMPU6050Task, "MPU6050 Task", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(vWiFiServerTask, "WiFi Server Task", 4096, NULL, 1, NULL, 0);
    // Start the scheduler
    vTaskStartScheduler();
}

void loop()
{
    // Empty loop as FreeRTOS handles task scheduling
}