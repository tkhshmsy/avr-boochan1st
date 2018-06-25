#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <Arduino.h>
#include "steppers.h"
#include "fan.h"

class Robot
{
public:
    enum BallColor{
        Unknown = 0,
        Red = 1,
        Yellow = 2,
        Blue = 3,
    };

    Robot(Steppers* steppers, Fan* fan);
    void controlFan(bool isActive);
    void controlArm(bool isClosed);
    bool isTimeCritical(); //for sensing-control

    //sensors
    void setLineSensor(uint8_t value);
    void setTofFront(uint16_t value);
    void setTofLeft(uint16_t value);
    void setTofRight(uint16_t value);
    void setColor(uint16_t red, uint16_t green, uint16_t blue, uint16_t clear);

    // bool isWaitingBallCheck();
    bool isGetBall();
    BallColor checkBallColor();

    //globals
    void status(); //output for Serial
    void exec(); //no return

    //for test
    void partAdjust1(bool onLine = true);
    void partAdjust2(bool onLine = true);
    void partForward(int16_t mm, bool accel = true);
    void partBackward(int16_t mm);
    void partTurnLeft(int16_t degree, bool accel = true, bool isSmall = true);
    void partTurnRight(int16_t degree, bool accel = true, bool isSmall = true);

    //for game
    //Start: 180CW
    void StartToFirstLine(); //from StartArea:180CW to OnFirstLine:180CW
    void FirstLineToSecondLine(); //from OnFirstLine:180CW to OnSecondLine:180CW
    void SecondLineToThirdLine(); //from OnSecondLine:180CW to OnThirdLine:270CW
    //if ball is near BallLine, to get it and return true
    bool ThirdLineToBallLine(); //from OnThirdLine:270CW to OFF BallLine:180CW
    
    void BallLineToThirdLine(); //from OFF BallLine:180CW to OnThirdLine:90CW
    void ThirdLineToSecondLine(); //from OnThirdLine:90CW to OnSecondLine:180CW
    void SecondLineToFirstLine(); //from OnSecondLine:180CW to OnFirstLine:180CW
    void FirstSecondGoal(); //from/to OnFirstLine:180CW
    void ThirdGoal(); //from OnThirdLine:90CW to OnThirdLine:270CW

    void FirstLineToZeroLine(); //from OnFirstLine:180CW to ZeroLine:90CW
    void ZeroLineToFirstLine(); //from ZeroLine:90CW to OnFirstLine:180CW

    bool TryToGet();
    bool HeadingToGetAndRecovery(int degree); //turn left is PLUS
    int HeadingSearch(int start = 60, int end = -60);
    bool SideSearchToGet(uint16_t sideLimit = 400, int counterMax = 220);

private:
    //routines
    bool adjustLineFrontForward1(bool onLine = true);
    bool adjustLineFrontForward2(bool onLine = true);

    long mm2Steps(int value);
    long deg2StepsSmall(int value);
    void waitingBallCheck();

    //variables
    Steppers *m_steppers;
    Fan *m_fan;

    //sensors
    volatile uint8_t m_lineSensor;

    volatile uint16_t m_tofFront;
    volatile uint16_t m_tofLeft;
    volatile uint16_t m_tofRight;

    // volatile bool m_isWaitingBallCheck;
    volatile uint16_t m_redColor;
    volatile uint16_t m_greenColor;
    volatile uint16_t m_blueColor;
    volatile uint16_t m_clearColor;
    static int m_colorSpace[3][3];
};

#endif //__ROBOT_H__