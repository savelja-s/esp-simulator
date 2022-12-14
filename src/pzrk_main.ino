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

const char *ssid = "Zhu";
const char *password = "13579135";
IPAddress addr(192, 168, 0, 29);
const uint16_t port = 49152;
AsyncUDP udp;
#define DEBUG_MODE 1
#define LISTEN_PORT 8888

long lastDebounceTime = 0; // the last time the output pin was toggled
long debounceDelay = 50;   // the debounce time; increase if the output flickers

// const int buttonPin = 23;
// const int HALL_1_PIN = 35;
// const int hall1pin = 18;
// buttons
#define BUTTON_1 23
#define BUTTON_2 1
#define BUTTON_3 2
#define BUTTON_4 3
#define BUTTON_5 4
#define BUTTON_6 5
#define BUTTON_7 6
#define BUTTON_8 7
#define BUTTON_9 8
#define BUTTON_10 9
// halls
#define HALL_1_PIN 18
#define HALL_2_PIN 1
#define HALL_3_PIN 2
#define HALL_4_PIN 3

ezButton button1(BUTTON_1);
ezButton button2(BUTTON_2);
ezButton button3(BUTTON_3);
ezButton button4(BUTTON_4);
ezButton button5(BUTTON_5);
ezButton button6(BUTTON_6);
ezButton button7(BUTTON_7);
ezButton button8(BUTTON_8);
ezButton button9(BUTTON_9);
ezButton button10(BUTTON_10);
ezButton hall1(HALL_1_PIN);
ezButton hall2(HALL_2_PIN);
ezButton hall3(HALL_3_PIN);
ezButton hall4(HALL_4_PIN);

uint8_t btn_prev;

void parsePacket(AsyncUDPPacket packet)
{
    // Выводи в последовательный порт все полученные данные
    Serial.write(packet.data(), packet.length());
    Serial.println();
    DynamicJsonDocument doc(packet.length());
    deserializeJson(doc, packet.data());
    if (doc.containsKey("soundID") && doc.containsKey("status"))
    {
        String status = doc["status"];
        String soundID = doc["soundID"];
        Serial.println("Sound " + status + " with ID " + soundID + ".Need add logic");
    }
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
    udp.print(packet.Serialize().c_str());
}
void loopButton(int pin, ezButton button, uint8_t type)
{
    button.loop(); // MUST call the loop() function first
    if (button.isPressed())
    {
#ifdef DEBUG_MODE
        Serial.println("The " + String(type) + " " + String(pin) + " is pressed");
#endif
        sendByUDP(type, createJsonButton(pin, 1));
    }
    if (button.isReleased())
    {
#ifdef DEBUG_MODE
        Serial.println("The " + String(type) + " " + String(pin) + " is released");
#endif
        sendByUDP(type, createJsonButton(pin, 0));
    }
}

void setupButton(int pin, ezButton button)
{
    button1.setDebounceTime(debounceDelay); // set debounce time to 50 milliseconds
    pinMode(pin, INPUT_PULLUP);
}

void setup()
{
    setupButton(BUTTON_1, button1);
    setupButton(BUTTON_2, button2);
    setupButton(BUTTON_3, button3);
    setupButton(BUTTON_4, button4);
    setupButton(BUTTON_5, button5);
    setupButton(BUTTON_6, button6);
    setupButton(BUTTON_7, button7);
    setupButton(BUTTON_8, button8);
    setupButton(BUTTON_9, button9);
    setupButton(BUTTON_10, button10);
    setupButton(HALL_1_PIN, hall1);
    setupButton(HALL_2_PIN, hall2);
    setupButton(HALL_3_PIN, hall3);
    setupButton(HALL_4_PIN, hall4);
    // button1.setDebounceTime(debounceDelay); // set debounce time to 50 milliseconds
    // hall1.setDebounceTime(debounceDelay);   // set debounce time to 50 milliseconds
    //     pinMode(HALL_1_PIN, INPUT);
    // pinMode(BUTTON_1, INPUT_PULLUP);
    // pinMode(hall1pin, INPUT_PULLUP);
    // btn_prev = digitalRead(BUTTON_1);

    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial)
        ;

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // // wait for ready
    // Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    // while (Serial.available() && Serial.read())
    //     ; // empty buffer
    // while (!Serial.available())
    //     ; // wait for data
    // while (Serial.available() && Serial.read())
    //     ; // empty buffer again

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

    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);

    // Ждём подключения WiFi
    while (WiFi.waitForConnectResult() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(100);
    }

    // Если удалось подключится по UDP
    if (udp.connect(addr, port))
    {

        Serial.println("UDP connected");

        // Call calback on recive
        // udp.onPacket(parsePacket);
    }
    else
    {

        Serial.println("UDP not connected");

        // Infinite cycle. TODO
        while (1)
        {
            delay(1000);
        }
    }
    if (udp.listen(LISTEN_PORT))
    {
        udp.onPacket(parsePacket);
        // udp.onPacket([](AsyncUDPPacket packet)
        //              {
        //     Serial.print("Data: ");
        //     Serial.write(packet.data(), packet.length());
        //     Serial.println(); });
    }
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop()
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
    loopButton(BUTTON_1, button1, PACKET_BUTTON);
    loopButton(BUTTON_2, button2, PACKET_BUTTON);
    loopButton(BUTTON_3, button3, PACKET_BUTTON);
    loopButton(BUTTON_4, button4, PACKET_BUTTON);
    loopButton(BUTTON_5, button5, PACKET_BUTTON);
    loopButton(BUTTON_6, button6, PACKET_BUTTON);
    loopButton(BUTTON_7, button7, PACKET_BUTTON);
    loopButton(BUTTON_8, button8, PACKET_BUTTON);
    loopButton(BUTTON_9, button9, PACKET_BUTTON);
    loopButton(BUTTON_10, button10, PACKET_BUTTON);
    // button1.loop(); // MUST call the loop() function first
    // hall1.loop(); // MUST call the loop() function first
    loopButton(HALL_1_PIN, hall1, PACKET_HALL);
    loopButton(HALL_2_PIN, hall2, PACKET_HALL);
    loopButton(HALL_3_PIN, hall3, PACKET_HALL);
    loopButton(HALL_4_PIN, hall4, PACKET_HALL);

    // if (button1.isPressed())
    // {
    //     Serial.println("The button 1 is pressed");
    //     packet.Create(packet.PACKET_BUTTON, createJsonButton(BUTTON_1, 1));
    //     // packet.Create(buttonPin, std::string("{ \"Button23\": 1 }"));
    //     udp.print(packet.Serialize().c_str());
    // }

    // if (button1.isReleased())
    // {
    //     Serial.println("The button 1 is released");
    //     // packet.Create(buttonPin, std::string("{ \"Button23\": 0 }"));
    //     packet.Create(packet.PACKET_BUTTON, createJsonButton(BUTTON_1, 0));
    //     udp.print(packet.Serialize().c_str());
    // }
    // if (hall1.isPressed())
    // {
    //     Serial.println("The hall1 is pressed");
    //     packet.Create(hall1pin, std::string("{ \"hall1\": 1 }"));
    //     udp.print(packet.Serialize().c_str());
    // }

    // if (hall1.isReleased())
    // {
    //     Serial.println("The hall1 is released");
    //     packet.Create(hall1pin, std::string("{ \"hall1\": 0 }"));
    //     udp.print(packet.Serialize().c_str());
    // }

    // //ANALOG READ EXAMPLE
    // uint16_t analogVal = analogRead(HALL_1_PIN);
    // std::ostringstream ss;
    // ss << "{ \"hall1_state\": " << analogVal << " }";
    // std::string body = ss.str();
    // packet.Create(HALL_1_PIN, body);
    // udp.print(packet.Serialize().c_str());
    // delay(1000);
    /////
}
