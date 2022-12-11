// // https://github.com/jrowberg/i2cdevlib
// #include "Wire.h"
// #include "MPU6050_6Axis_MotionApps612.h"
// #include "I2Cdev.h"
// #define OUTPUT_READABLE_QUATERNION
// #define INTERRUPT_PIN 15
// #define LED_PIN 13
// bool blinkState = false;
// MPU6050 mpu;

// // MPU control/status vars
// bool dmpReady = false;  // set true if DMP init was successful
// uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
// uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
// uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
// uint16_t fifoCount;     // count of all bytes currently in FIFO
// uint8_t fifoBuffer[64]; // FIFO storage buffer

// // orientation/motion vars
// Quaternion q;        // [w, x, y, z]         quaternion container
// VectorInt16 aa;      // [x, y, z]            accel sensor measurements
// VectorInt16 gy;      // [x, y, z]            gyro sensor measurements
// VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
// VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
// VectorFloat gravity; // [x, y, z]            gravity vector
// float euler[3];      // [psi, theta, phi]    Euler angle container
// float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// // packet structure for InvenSense teapot demo
// uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};

// // ================================================================
// // ===               INTERRUPT DETECTION ROUTINE                ===
// // ================================================================

// volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
// void dmpDataReady()
// {
//     mpuInterrupt = true;
// }

// void begin_mpu()
// {
//     Wire.begin();
//     Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

//     // initialize serial communication
//     // (115200 chosen because it is required for Teapot Demo output, but it's
//     // really up to you depending on your project)
//     Serial.begin(115200);
//     while (!Serial); // wait for Leonardo enumeration, others continue immediately

//     // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
//     // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
//     // the baud timing being too misaligned with processor ticks. You must use
//     // 38400 or slower in these cases, or use some kind of external separate
//     // crystal solution for the UART timer.

//     // initialize device
//     Serial.println(F("Initializing I2C devices..."));
//     mpu.initialize();
//     pinMode(INTERRUPT_PIN, INPUT);

//     // verify connection
//     Serial.println(F("Testing device connections..."));
//     Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

//     // wait for ready
//     Serial.println(F("\nSend any character to begin DMP programming and demo: "));
//     while (Serial.available() && Serial.read())
//         ; // empty buffer
//     while (!Serial.available())
//         ; // wait for data
//     while (Serial.available() && Serial.read())
//         ; // empty buffer again

//     // load and configure the DMP
//     Serial.println(F("Initializing DMP..."));
//     devStatus = mpu.dmpInitialize();

//     // supply your own gyro offsets here, scaled for min sensitivity
//     mpu.setXGyroOffset(51);
//     mpu.setYGyroOffset(8);
//     mpu.setZGyroOffset(21);
//     mpu.setXAccelOffset(1150);
//     mpu.setYAccelOffset(-50);
//     mpu.setZAccelOffset(1060);
//     // make sure it worked (returns 0 if so)
//     if (devStatus == 0)
//     {
//         // Calibration Time: generate offsets and calibrate our MPU6050
//         mpu.CalibrateAccel(6);
//         mpu.CalibrateGyro(6);
//         Serial.println();
//         mpu.PrintActiveOffsets();
//         // turn on the DMP, now that it's ready
//         Serial.println(F("Enabling DMP..."));
//         mpu.setDMPEnabled(true);

//         // enable Arduino interrupt detection
//         Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
//         Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
//         Serial.println(F(")..."));
//         attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
//         mpuIntStatus = mpu.getIntStatus();

//         // set our DMP Ready flag so the main loop() function knows it's okay to use it
//         Serial.println(F("DMP ready! Waiting for first interrupt..."));
//         dmpReady = true;

//         // get expected DMP packet size for later comparison
//         packetSize = mpu.dmpGetFIFOPacketSize();
//     }
//     else
//     {
//         // ERROR!
//         // 1 = initial memory load failed
//         // 2 = DMP configuration updates failed
//         // (if it's going to break, usually the code will be 1)
//         Serial.print(F("DMP Initialization failed (code "));
//         Serial.print(devStatus);
//         Serial.println(F(")"));
//     }
// }

// void get_mpu()
// {
//     // if programming failed, don't try to do anything
//     if (!dmpReady)
//         return;
//     // read a packet from FIFO
//     if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
//     { // Get the Latest packet

// #ifdef OUTPUT_READABLE_QUATERNION
//         // display quaternion values in easy matrix form: w x y z
//         mpu.dmpGetQuaternion(&q, fifoBuffer);
//         // Serial.print("quat\t");
//         // Serial.print(q.w);
//         // Serial.print("\t");
//         // Serial.print(q.x);
//         // Serial.print("\t");
//         // Serial.print(q.y);
//         // Serial.print("\t");
//         // Serial.println(q.z);
// #endif

// #ifdef OUTPUT_TEAPOT
//         // display quaternion values in InvenSense Teapot demo format:
//         teapotPacket[2] = fifoBuffer[0];
//         teapotPacket[3] = fifoBuffer[1];
//         teapotPacket[4] = fifoBuffer[4];
//         teapotPacket[5] = fifoBuffer[5];
//         teapotPacket[6] = fifoBuffer[8];
//         teapotPacket[7] = fifoBuffer[9];
//         teapotPacket[8] = fifoBuffer[12];
//         teapotPacket[9] = fifoBuffer[13];
//         Serial.write(teapotPacket, 14);
//         teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
// #endif

//         // blink LED to indicate activity
//         blinkState = !blinkState;
//         digitalWrite(LED_PIN, blinkState);
//     }
// }