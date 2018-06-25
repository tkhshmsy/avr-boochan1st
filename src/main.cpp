#include <Arduino.h>
#include <FlexiTimer2.h>

#define __VL53L0X__
#define __VL53L0X_ALL__
#define __TCS34725__

#ifdef __TCS34725__
#include <TI_TCA9548A.h>
// #include <Adafruit_TCS34725.h>
#include <Adafruit_TCS34725softi2c/Adafruit_TCS34725softi2c.h>
#endif
#ifdef __VL53L0X__
#include <VL53L0X.h>
#endif

#include "steppers.h"
#include "fan.h"
#include "robot.h"

#define GOSW (2)
#define SCLpin (3) //for SoftwareWire
#define SDApin (4) //for SoftwareWire
#define FAN_FAN (10)
#define FAN_ARM (9)
#define LED_RED (8)
#define LED_YELLOW (7)
#define LS_RIGHT (11)
#define LS_LEFT (12)

#define LED_ON (0)
#define LED_OFF (1)

TI_TCA9548A tca9548a(&Wire);
#define CH_TCS34725 (0)
#define CH_VL53FRONT (7)
#define CH_VL53LEFT (6)
#define CH_VL53RIGHT (5)

#ifdef __TCS34725__
// Adafruit_TCS34725 tcs34725 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS,
//                                         TCS34725_GAIN_16X);
Adafruit_TCS34725softi2c tcs34725 = Adafruit_TCS34725softi2c(TCS34725_INTEGRATIONTIME_2_4MS,
                                                        TCS34725_GAIN_16X,
                                                        SDApin,
                                                        SCLpin);
#endif
#ifdef __VL53L0X__
VL53L0X vl53Front;
VL53L0X vl53Left;
VL53L0X vl53Right;
#endif

Servo servo;
Steppers steppers(14, 15, 16, 17);
Fan fan(FAN_FAN);
Robot robot(&steppers, &fan);

volatile uint8_t liveCounter = 0;
volatile bool live = false;
volatile bool isInISR = false;

void timerLoop()
{
    if (isInISR) {
        return;
    }
    isInISR = true;

    //enable ISR for i2c
    sei();

    //sensing
    uint8_t ls = 0x00;
    if (digitalRead(LS_LEFT)) {
        ls |= 0x02;
    }
    if (digitalRead(LS_RIGHT)) {
        ls |= 0x01;
    }
    robot.setLineSensor(ls);

    if (robot.isTimeCritical() == false) {
        static uint8_t selector = 0;
#ifdef __TCS34725__
        // if (robot.isWaitingBallCheck()) {
            if (selector == 0) {
                uint16_t r = 0;
                uint16_t g = 0;
                uint16_t b = 0;
                uint16_t c = 0;
                tca9548a.selectChannel(CH_TCS34725);
                tcs34725.getRawData(&r, &g, &b, &c);
                robot.setColor(r,g,b,c);
            }
        // }
#endif
#ifdef __VL53L0X__
        if (selector == 1) {
            tca9548a.selectChannel(CH_VL53FRONT);
            if ((vl53Front.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) != 0) {
                uint16_t range = vl53Front.readRangeContinuousMillimeters();
                if (!vl53Front.timeoutOccurred()) {
                    robot.setTofFront(range);
                } else {
                    //timed out
                    Serial.println("ToFF T/O");                
                    vl53Front.init();
                    vl53Front.setTimeout(250);
                    vl53Front.startContinuous();
                }
            }
#ifdef __VL53L0X_ALL__
        } else if(selector == 2) {
            tca9548a.selectChannel(CH_VL53LEFT);
            if ((vl53Left.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) != 0) {
                uint16_t range = vl53Left.readRangeContinuousMillimeters();
                if (!vl53Left.timeoutOccurred()) {
                    robot.setTofLeft(range);
                } else {
                    //timed out
                    Serial.println("ToFL T/O");
                    vl53Left.init();
                    vl53Left.setTimeout(250);
                    vl53Left.startContinuous();
                }
            }
        } else if (selector == 3) {
            tca9548a.selectChannel(CH_VL53RIGHT);
            if ((vl53Right.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) != 0) {
                uint16_t range = vl53Right.readRangeContinuousMillimeters();
                if (!vl53Right.timeoutOccurred()) {
                    robot.setTofRight(range);
                } else {
                    //timed out
                    Serial.println("ToFR T/O");
                    vl53Right.init();
                    vl53Right.setTimeout(250);
                    vl53Right.startContinuous();
                }
            }
#endif
        }
#endif

#ifdef __VL53L0X__
        selector++;
#ifdef __VL53L0X_ALL__
        if (selector > 3) {
#else
        if (selector > 1) {
#endif
            selector = 0;
        }
#endif
    }
    //live beacon
    liveCounter++;
    if (liveCounter % 100 == 0) {
        digitalWrite(LED_BUILTIN, live);
        live = !live;
        liveCounter = 0;

        if (robot.isGetBall()) {
            Robot::BallColor ballColor = robot.checkBallColor();
            if (ballColor == Robot::BallColor::Red) {
                digitalWrite(LED_RED, LED_ON);
                digitalWrite(LED_YELLOW, LED_OFF);
            } else if (ballColor == Robot::BallColor::Yellow) {
                digitalWrite(LED_RED, LED_OFF);
                digitalWrite(LED_YELLOW, LED_ON);
            } else if (ballColor == Robot::BallColor::Blue) {
                digitalWrite(LED_RED, LED_ON);
                digitalWrite(LED_YELLOW, LED_ON);
            } else {
                digitalWrite(LED_RED, LED_OFF);
                digitalWrite(LED_YELLOW, LED_OFF);                
            }
        } else {
            digitalWrite(LED_RED, LED_OFF);
            digitalWrite(LED_YELLOW, LED_OFF);    
        }
        
        if (robot.isTimeCritical() == false) {
            robot.status();
        }
    }

    //exit ISR
    isInISR = false;
}

void setup()
{
    Serial.begin(38400);
    Wire.begin();
    Wire.setClock(400000L);

    //live beacon
    pinMode(LED_BUILTIN, OUTPUT);
    //go switch
    pinMode(GOSW, INPUT_PULLUP);
    //linesensor
    pinMode(LS_LEFT, INPUT);
    pinMode(LS_RIGHT, INPUT);
    // led
    pinMode(LED_RED, OUTPUT);
    digitalWrite(LED_RED, LED_OFF);
    pinMode(LED_YELLOW, OUTPUT);
    digitalWrite(LED_YELLOW, LED_OFF);
    // arm
    servo.attach(9);
    fan.setArmServo(&servo);
    fan.setClose(false);

    //i2c
#ifdef __VL53L0X__
    Serial.print("VL53L0X Front setting...");
    delay(150);
    tca9548a.selectChannel(CH_VL53FRONT);
    delay(150);
    Serial.print("init...");
    if(!vl53Front.init(true)){
        Serial.println("fail...");
    }
    delay(150);
    vl53Front.setTimeout(250);
    vl53Front.startContinuous();
    Serial.println("OK");

#ifdef __VL53L0X_ALL__
    Serial.print("VL53L0X Left setting...");
    delay(150);
    tca9548a.selectChannel(CH_VL53LEFT);
    delay(150);
    Serial.print("init...");
    if(!vl53Left.init(true)) {
        Serial.println("fail...");
    }
    delay(150);
    vl53Left.setTimeout(250);
    vl53Left.startContinuous();
    Serial.println("OK");

    Serial.print("VL53L0X Right setting...");
    delay(150);
    tca9548a.selectChannel(CH_VL53RIGHT);
    delay(150);
    Serial.print("init...");
    if(!vl53Right.init(true)) {
        Serial.println("fail...");
    }
    delay(150);
    vl53Right.setTimeout(250);
    vl53Right.startContinuous();
    Serial.println("OK");
#endif
#endif
#ifdef __TCS34725__
    Serial.print("TCS34725 setting...");
    tca9548a.selectChannel(CH_TCS34725);
    if (tcs34725.begin()) {
        Serial.println("OK");
    } else {
        Serial.println("no TCS34725");
        while(1);
    }
#endif

    //timer
    isInISR = false;
    FlexiTimer2::set(5, 1.0/1000, timerLoop); //50ms
    FlexiTimer2::start();
}

void loop()
{
    Serial.println("**** GO WAIT ****");
    while (digitalRead(GOSW) == 1) {
        digitalWrite(LED_RED, live);
        digitalWrite(LED_YELLOW, !live);
    }
    Serial.println("**** READY ****");
    delay(1000);
    Serial.println("**** START ****");

    //main seq
    robot.exec();

    //unreachable
    Serial.println("**** UNREACHABLE ****\n");
    while(1);
}