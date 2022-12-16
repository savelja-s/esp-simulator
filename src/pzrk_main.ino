// https://github.com/jrowberg/i2cdevlib

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "WiFi.h"
#include "AsyncUDP.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "ezButton.h"
#include <ArduinoJson.h>
#include <sstream>
#include "packet.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// const char *ssid = "TheNest_2.4";
// const char *password = "ibhjrf2939";
// IPAddress addr(192, 168, 31, 218);
const char *ssid = "Zhu";
const char *password = "13579135";
IPAddress addr(192, 168, 0, 29);

const uint16_t port = 49152;
AsyncUDP udp;
#define DEBUG_MODE 1
#define LISTEN_PORT 8888

long lastDebounceTime = 0; // the last time the output pin was toggled
long debounceDelay = 50;   // the debounce time; increase if the output flickers

// buttons
#define BUTTON_1_PIN 13
#define BUTTON_2_PIN 12
#define BUTTON_3_PIN 14
#define BUTTON_4_PIN 27
#define BUTTON_5_PIN 26
#define BUTTON_6_PIN 25
#define BUTTON_7_PIN 33
#define BUTTON_8_PIN 32
#define BUTTON_9_PIN 35
#define BUTTON_10_PIN 34
#define BUTTON_11_PIN 39
#define BUTTON_12_PIN 36

ezButton button1(BUTTON_1_PIN);
ezButton button2(BUTTON_2_PIN);
ezButton button3(BUTTON_3_PIN);
ezButton button4(BUTTON_4_PIN);
ezButton button5(BUTTON_5_PIN);
ezButton button6(BUTTON_6_PIN);
ezButton button7(BUTTON_7_PIN);
ezButton button8(BUTTON_8_PIN);
ezButton button9(BUTTON_9_PIN);
ezButton button10(BUTTON_10_PIN);
ezButton button11(BUTTON_11_PIN);
ezButton button12(BUTTON_12_PIN);

// uint8_t btn_prev;

void parsePacket(AsyncUDPPacket packet)
{
    // Выводи в последовательный порт все полученные данные
    Serial.write(packet.data(), packet.length());
    Serial.println("----------------_____________----------");
    std::stringstream ss;
    ss << packet.data();
    Serial.println("----------------_____________----------");
    Serial.println(ss.str().c_str());
    Serial.println("----------------_____________----------");
    Packet packetM;
    packetM.DeSerialize(ss.str());
    Serial.println(packetM.Serialize().c_str());
    // DynamicJsonDocument doc(packet.length());
    // deserializeJson(doc, packet.data());
    // String status = doc["status"];
    // String soundID = doc["soundID"];
    //  Serial.println(status);
    // if (doc.containsKey("soundID") && doc.containsKey("status"))
    // {
    //     String status = doc["status"];
    //     String soundID = doc["soundID"];
    //     Serial.println("Sound " + status + " with ID " + soundID + ".Need add logic");
    // }
    packet.println("OK");
    packet.flush();
}

// class default I2C address is 0x68

MPU6050 mpu;

#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
// #define OUTPUT_TEAPOT

#define INTERRUPT_PIN 19 // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13       // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q; // [w, x, y, z]         quaternion container

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

// ================================================================
// ===                       CREATE JSON                        ===
// ================================================================

String create_quanternion_json(float w, float x, float y, float z)
{
    DynamicJsonDocument doc(1024);
    doc["q.w"] = w;
    doc["q.x"] = x;
    doc["q.y"] = y;
    doc["q.z"] = z;
    String output;
    serializeJson(doc, output);
    return output;
}

std::string createJsonButton(int buttonPIN, int status)
{
    DynamicJsonDocument doc(1024);
    doc["buttonId"] = buttonPIN;
    doc["status"] = status;

    std::string output;
    serializeJson(doc, output);
    return output;
}

std::string createJsonEncoder(int encoderPIN, int value)
{
    DynamicJsonDocument doc(1024);
    doc["encoderId"] = encoderPIN;
    doc["value"] = value;

    std::string output;
    serializeJson(doc, output);
    return output;
}
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void sendByUDP(uint8_t type, std::string body)
{
    Packet packet;
    packet.Create(type, body);
    udp.connect(addr, port);
    udp.print(packet.Serialize().c_str());
    udp.close();
}

void loopButton(int pin, ezButton &button, uint8_t type)
{
    button.loop(); // MUST call the loop() function first
    if (button.isPressed())
    {
        if (DEBUG_MODE)
        {
            Serial.println("The " + String(type) + " " + String(pin) + " is pressed");
        }
        sendByUDP(type, createJsonButton(pin, 1));
    }
    if (button.isReleased())
    {
        if (DEBUG_MODE)
        {
            Serial.println("The " + String(type) + " " + String(pin) + " is released");
        }
        sendByUDP(type, createJsonButton(pin, 0));
    }
}

void setupButton(int pin, ezButton button)
{
    button.setDebounceTime(debounceDelay); // set debounce time to 50 milliseconds
    pinMode(pin, INPUT_PULLUP);
}

void initMPU()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(51);
    mpu.setYGyroOffset(8);
    mpu.setZGyroOffset(21);
    mpu.setXAccelOffset(1150);
    mpu.setYAccelOffset(-50);
    mpu.setZAccelOffset(1060);
    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        Serial.println();
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}

void initWIFI()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    // Ждём подключения WiFi
    while (WiFi.waitForConnectResult() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(100);
    }
}
void initUDP()
{
    if (udp.listen(LISTEN_PORT))
    {
        Serial.print("[UDP] Listen IP:");
        Serial.print(WiFi.localIP().toString().c_str());
        Serial.print(" PORT:");
        Serial.println(LISTEN_PORT);
        udp.onPacket(parsePacket);
    }
}

void setup()
{
    setupButton(BUTTON_1_PIN, button1);
    setupButton(BUTTON_2_PIN, button2);
    setupButton(BUTTON_3_PIN, button3);
    setupButton(BUTTON_4_PIN, button4);
    setupButton(BUTTON_5_PIN, button5);
    setupButton(BUTTON_6_PIN, button6);
    setupButton(BUTTON_7_PIN, button7);
    setupButton(BUTTON_8_PIN, button8);
    setupButton(BUTTON_9_PIN, button9);
    setupButton(BUTTON_10_PIN, button10);
    setupButton(BUTTON_11_PIN, button11);
    setupButton(BUTTON_12_PIN, button12);

    Serial.begin(115200);
    while (!Serial)
        ;
    initMPU();

    initWIFI();

    initUDP();
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loopMPU()
{
    // if programming failed, don't try to do anything
    if (!dmpReady)
        return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    { // Get the Latest packet

#ifdef OUTPUT_READABLE_QUATERNION
      // display quaternion values in easy matrix form: w x y z
        mpu.dmpGetQuaternion(&q, fifoBuffer);
#endif

#ifdef OUTPUT_TEAPOT
        // display quaternion values in InvenSense Teapot demo format:
        teapotPacket[2] = fifoBuffer[0];
        teapotPacket[3] = fifoBuffer[1];
        teapotPacket[4] = fifoBuffer[4];
        teapotPacket[5] = fifoBuffer[5];
        teapotPacket[6] = fifoBuffer[8];
        teapotPacket[7] = fifoBuffer[9];
        teapotPacket[8] = fifoBuffer[12];
        teapotPacket[9] = fifoBuffer[13];
        Serial.write(teapotPacket, 14);
        teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
#endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
    if (WiFi.status() != WL_CONNECTED)
    {
        // Вызываем функцию setup(), для повторного подключения
        setup();
    }
}
void loop()
{
    loopMPU();
    loopButton(BUTTON_1_PIN, button1, PACKET_BUTTON);
    loopButton(BUTTON_2_PIN, button2, PACKET_BUTTON);
    loopButton(BUTTON_3_PIN, button3, PACKET_BUTTON);
    loopButton(BUTTON_4_PIN, button4, PACKET_BUTTON);
    loopButton(BUTTON_5_PIN, button5, PACKET_BUTTON);
    loopButton(BUTTON_6_PIN, button6, PACKET_BUTTON);
    loopButton(BUTTON_7_PIN, button7, PACKET_BUTTON);
    loopButton(BUTTON_8_PIN, button8, PACKET_BUTTON);
    loopButton(BUTTON_9_PIN, button9, PACKET_BUTTON);
    loopButton(BUTTON_10_PIN, button10, PACKET_BUTTON);
    loopButton(BUTTON_11_PIN, button11, PACKET_BUTTON);
    loopButton(BUTTON_12_PIN, button12, PACKET_BUTTON);
}
