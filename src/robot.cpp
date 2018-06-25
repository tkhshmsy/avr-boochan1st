#include "robot.h"

//Stable
#define ST_STEPS (400) // 1round 400steps(fullstep)
#define ST_DIAMETER (62) // raduis(mm)
#define ST_WHEELBASE (140) //wheelbase(mm)
#define ST_LS2WHEELBASE (45) //length(mm) linesensorFront to wheelbase
#define ST_SAFETY (40) //length(mm) safty length for adjusting
#define ST_SAFETY_LARGE (ST_LS2WHEELBASE + ST_SAFETY)

//LineSensor
#define LS_FrontLeft (0x02)
#define LS_FrontRight (0x01)
#define LS_FrontBoth (LS_FrontLeft | LS_FrontRight)
//ToFSensor
#define TOF_SHORT_LIMIT (50)
#define TOF_LONG_LIMIT (300)
#define TOF_LONG2_LIMIT (200)
#define TOF_SHORT2_LIMIT (150)
#define TOF_NONE (60000)

//BallSearch
#define BS_FAIL (255)
#define BS_SearchEveryMM (80) // mm/search
#define BS_SearchCounterMax (20)

//BallSearch - TryToGet
#define BS_BallCheckEveryMM (5) // mm/check
#define BS_BallCheckCounterMax (40)
//BallSearch - SideSearch
#define BS_SideBallCheckCounterMax_Long (220)
#define BS_SideBallCheckCounterMax_Short (120)

int Robot::m_colorSpace[3][3] = {
    {10, 10, 10},
    {20, 20, 20},
    {30, 30, 30},
};

Robot::Robot(Steppers *steppers, Fan *fan)
    : m_steppers(steppers)
    , m_fan(fan)
    , m_lineSensor(0)
    , m_tofFront(0)
    , m_tofLeft(0)
    , m_tofRight(0)
    // , m_isWaitingBallCheck(false)
    , m_redColor(0)
    , m_greenColor(0)
    , m_blueColor(0)
    , m_clearColor(0)
{}

void Robot::setLineSensor(uint8_t value)
{
    m_lineSensor = value;
}

void Robot::setTofFront(uint16_t value)
{
    if (value < TOF_SHORT_LIMIT
        || value > TOF_LONG_LIMIT) {
        m_tofFront = TOF_NONE;
        return;
    }
    m_tofFront = value;
}

void Robot::setTofLeft(uint16_t value)
{
    if (value < TOF_SHORT_LIMIT
        || value > TOF_LONG_LIMIT) {
        m_tofLeft = TOF_NONE;
        return;
    }

    m_tofLeft = value;
}

void Robot::setTofRight(uint16_t value)
{
    if (value < TOF_SHORT_LIMIT
        || value > TOF_LONG_LIMIT) {
        m_tofRight = TOF_NONE;
        return;
    }
    m_tofRight = value;
}

void Robot::setColor(uint16_t red, uint16_t green, uint16_t blue, uint16_t clear)
{
    static int counter = 0;
    static uint16_t reds[10];
    static uint16_t greens[10];
    static uint16_t blues[10];
    static uint16_t clears[10];

    reds[counter] = red;
    greens[counter] = green;
    blues[counter] = blue;
    clears[counter] = clear;
    counter++;
    if (counter > 10) {
        counter = 0;    
    }

    uint16_t sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += reds[i];
    }
    m_redColor = sum / 10;
    sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += greens[i];
    }
    m_greenColor = sum / 10;
    sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += blues[i];
    }
    m_blueColor = sum / 10;
    sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += clears[i];
    }
    m_clearColor = sum / 10;
}

bool Robot::isGetBall()
{
    if (m_fan->active() == false) {
        return false;
    }
    if (m_clearColor < 200) {
        return true;
    }
    return false;
}

Robot::BallColor Robot::checkBallColor()
{
    if (!isGetBall()) {
        return BallColor::Unknown;
    }

    //make norm * 256
    uint16_t length = (uint16_t)sqrt(m_redColor * m_redColor
                                    + m_greenColor * m_greenColor
                                    + m_blueColor * m_blueColor);
    uint16_t norm[3];
    norm[0] = (m_redColor * 256) / length;
    norm[1] = (m_greenColor * 256) / length;
    norm[2] = (m_blueColor * 256) / length;
    
    //min check
    long tmp = 0;
    long min = 32768 << 16;
    int index = -1;
    for (int i = 0; i < 3; i++) {
        tmp = 0;
        for (int j = 0; j < 3; j++) {
            tmp = (m_colorSpace[i][j] - norm[j]) * (m_colorSpace[i][j] - norm[j]);
        }
        if (min > tmp) {
            min = tmp;
            index = i;
        }
    }
    switch (index) {
    case 0:
        return BallColor::Red;
    case 1:
        return BallColor::Yellow;
    case 2:
        return BallColor::Blue;
    default:
        return BallColor::Unknown;
    }
    return BallColor::Unknown;
}

bool Robot::isTimeCritical()
{
    return m_steppers->isAccelMoving();
}

long Robot::mm2Steps(int value)
{
    // value * (ST_STEPS / (ST_DIAMETER * PI))
    long ret = (ST_STEPS / (ST_DIAMETER * PI)) * value;
    // Serial.print("mm2step:");
    // Serial.println(ret, DEC);
    return ret;
}

long Robot::deg2StepsSmall(int value)
{
    // (ST_WHEELBASE * PI) * (ST_STEPS / (ST_DIAMETER * PI)) / 360 * value
    long ret = ((ST_WHEELBASE * PI) * (ST_STEPS / (ST_DIAMETER * PI)) / 360.0) * value;
    // Serial.print("deg2step:");
    // Serial.println(ret, DEC);
    return ret;
}

void Robot::status()
{
    static uint8_t counter = 0;
    Serial.print(counter, DEC);
    Serial.print("status: [");

    Serial.print(" ls=");
    Serial.print(m_lineSensor, HEX);

    Serial.print(" col=(");
    Serial.print(m_redColor, DEC);
    Serial.print(", ");
    Serial.print(m_greenColor, DEC);
    Serial.print(", ");
    Serial.print(m_blueColor, DEC);
    Serial.print(", ");
    Serial.print(m_clearColor, DEC);
    Serial.print(")");

    Serial.print(" bc=(");
    Serial.print(isGetBall(), DEC);
    Serial.print(", ");
    Serial.print((int)checkBallColor(), DEC);
    Serial.print(")");
    
    Serial.print(" ToFF=");
    Serial.print(m_tofFront, DEC);
    Serial.print(" ToFL=");
    Serial.print(m_tofLeft, DEC);
    Serial.print(" ToFR=");
    Serial.print(m_tofRight, DEC);

    Serial.print(" FAN=");
    Serial.print(m_fan->active(), DEC);

    Serial.println(" ]");
    counter++;
}

void Robot::controlFan(bool isActive)
{
    if (m_fan->active() != isActive) {
        m_fan->setActive(isActive);
    }
}

void Robot::controlArm(bool isClosed)
{
    if (m_fan->closed() != isClosed) {
        m_fan->setClose(isClosed);
    }
}

bool Robot::adjustLineFrontForward1(bool onLine)
{
    Serial.println("adjFF: Enter");
    Steppers::Direction left = Steppers::Direction::Forward;
    Steppers::Direction right = Steppers::Direction::Forward;
    // Serial.print("adjFF - 1 ");    Serial.println(m_lineSensor, HEX);
    while ((m_lineSensor & LS_FrontBoth) == 0) {
        m_steppers->move(left, right, 1, false);
        while(m_steppers->exec());
    }
    delay(100);
    // Serial.print("adjFF - 2 ");    Serial.println(m_lineSensor, HEX);
    if ((m_lineSensor & LS_FrontLeft) != 0 && (m_lineSensor & LS_FrontRight) != 0) {
        if (onLine) {
            Serial.println("adjFF: END onLINE");
            m_steppers->move(Steppers::Direction::Forward, Steppers::Direction::Forward,
                                mm2Steps(ST_LS2WHEELBASE), true);
            while(m_steppers->exec());
        } else {
            Serial.println("adjFF: END free");
        }
        return true;
    }
    // Serial.print("adjFF - 3 ");    Serial.println(m_lineSensor, HEX);
    left = (m_lineSensor & LS_FrontLeft) != 0 ? Steppers::Direction::Stop : Steppers::Direction::Forward;
    right =(m_lineSensor & LS_FrontRight) != 0 ? Steppers::Direction::Stop : Steppers::Direction::Forward;
    int counter = 0;    
    while ( !((m_lineSensor & LS_FrontLeft) != 0 && (m_lineSensor & LS_FrontRight) != 0) ) {
        m_steppers->move(left, right, 1, false);
        while(m_steppers->exec()) counter++;
    }
    Serial.print("adjFF - left:"); Serial.print((int)left, DEC);
    Serial.print(" step:"); Serial.println(counter, DEC);

    // Serial.print("adjFF - 4 ");    Serial.println(m_lineSensor, HEX);
    // while ( !((m_lineSensor & LS_FrontLeft) == 0 && (m_lineSensor & LS_FrontRight) == 0) ){
    while ( !((m_lineSensor & LS_FrontBoth) == 0) ) {
        m_steppers->move(Steppers::Direction::Backward, Steppers::Direction::Backward, 1, false);
        while(m_steppers->exec());
    }
    delay(100);
    // Serial.print("adjFF - 5 ");    Serial.println(m_lineSensor, HEX);
    m_steppers->move(Steppers::Direction::Backward, Steppers::Direction::Backward,
                         mm2Steps(ST_SAFETY), true);
    while(m_steppers->exec());

    // Serial.println("adjFF: exit");
    return false;
}

bool Robot::adjustLineFrontForward2(bool onLine)
{
    Serial.println("adjFF2: Enter");
    Steppers::Direction left = Steppers::Direction::Forward;
    Steppers::Direction right = Steppers::Direction::Forward;
    // Serial.print("adjFF - 1 ");    Serial.println(m_lineSensor, HEX);
    while ((m_lineSensor & LS_FrontBoth) == 0) {
        m_steppers->move(left, right, 1, false);
        while(m_steppers->exec());
    }
    delay(100);
    // Serial.print("adjFF2 - 2 ");    Serial.println(m_lineSensor, HEX);
    if ((m_lineSensor & LS_FrontLeft) != 0 && (m_lineSensor & LS_FrontRight) != 0) {
        if (onLine) {
            Serial.println("adjFF2: END onLINE");
            delay(1000);
            m_steppers->move(Steppers::Direction::Forward, Steppers::Direction::Forward,
                                mm2Steps(ST_LS2WHEELBASE), true);
            while(m_steppers->exec());
        } else {
            Serial.println("adjFF2: END free");
        }
        delay(1000);
        return true;
    }
    // Serial.print("adjFF2 - 3 ");    Serial.println(m_lineSensor, HEX);
    bool leftSensor = (m_lineSensor & LS_FrontLeft) != 0x00 ? true : false;
    bool rightSensor = (m_lineSensor & LS_FrontRight) != 0x00 ? true : false;
    // Serial.print("var:"); Serial.print((int)leftSensor); Serial.println((int)rightSensor);
    int counter = 0;
    while ( (leftSensor && ((m_lineSensor & LS_FrontRight) == 0))
        || (rightSensor && ((m_lineSensor & LS_FrontLeft) == 0))) {
            m_steppers->move(Steppers::Direction::Forward, Steppers::Direction::Forward,
                                1, false);
        while(m_steppers->exec());
        counter++;
        // Serial.print("var:"); Serial.print((int)leftSensor); Serial.println((int)rightSensor);
    }
    Serial.print("adjFF2 - left:"); Serial.print((int)leftSensor, DEC);
    Serial.print(" step:"); Serial.println(counter, DEC);

    if (leftSensor) {
        left = Steppers::Direction::Backward;
        // left = Steppers::Direction::Stop;
        right = Steppers::Direction::Forward;
    } else if (rightSensor) {
        left = Steppers::Direction::Forward;
        right = Steppers::Direction::Backward;        
        // right = Steppers::Direction::Stop;        
    } else {
        Serial.println("adjFF2: unreachable");
        while(1);
    }
    m_steppers->move(left, right, counter, false);
    while(m_steppers->exec());

    // Serial.print("adjFF2 - 4 ");    Serial.println(m_lineSensor, HEX);
    while ( !((m_lineSensor & LS_FrontBoth) == 0) ) {
        m_steppers->move(Steppers::Direction::Backward, Steppers::Direction::Backward, 1, false);
        while(m_steppers->exec());
    }
    delay(100);
    // Serial.print("adjFF2 - 5 ");    Serial.println(m_lineSensor, HEX);
    m_steppers->move(Steppers::Direction::Backward, Steppers::Direction::Backward,
                         mm2Steps(ST_SAFETY), false);
    while(m_steppers->exec());
    
    Serial.println("adjFF2: exit");
    return false;
}

void Robot::partAdjust1(bool onLine)
{
    Serial.println("part Adjust1");
    while(adjustLineFrontForward1(onLine) == false);
}

void Robot::partAdjust2(bool onLine)
{
    Serial.println("part Adjust2");
    while(adjustLineFrontForward2(onLine) == false);
}

void Robot::partForward(int16_t mm, bool accel)
{
    Serial.print("part Forward: ");
    Serial.println(mm);

    m_steppers->move(Steppers::Direction::Forward, Steppers::Direction::Forward,
                        mm2Steps(mm), accel);
    while(m_steppers->exec());
    if (accel) {
        delay(1000);
    }
}

void Robot::partBackward(int16_t mm)
{
    Serial.print("part Backward: ");
    Serial.println(mm);

    m_steppers->move(Steppers::Direction::Backward, Steppers::Direction::Backward,
                        mm2Steps(mm), true);
    while(m_steppers->exec());
    delay(1000);
}

void Robot::partTurnLeft(int16_t degree, bool accel, bool isSmall)
{
    int target = degree;
    Steppers::Direction left = Steppers::Direction::Backward;

    if (isSmall == true) {
        Serial.print("part TurnLeft: ");
    } else {
        Serial.print("part TurnLeft LARGE: ");
        target = degree * 2;
        left = Steppers::Direction::Stop;
    }
    Serial.println(degree);

    bool flag = isGetBall();
    m_steppers->move(left, Steppers::Direction::Forward, deg2StepsSmall(target), !flag);
    // m_steppers->move(left, Steppers::Direction::Forward, deg2StepsSmall(target), accel);
    while(m_steppers->exec());
    delay(1000);
}

void Robot::partTurnRight(int16_t degree, bool accel, bool isSmall)
{
    int target = degree;
    Steppers::Direction right = Steppers::Direction::Backward;

    if (isSmall == true) {
        Serial.print("part TurnRight: ");
    } else {
        Serial.print("part TurnRight LARGE: ");
        target = degree * 2;
        right = Steppers::Direction::Stop;
    }
    Serial.println(degree);
    
    bool flag = isGetBall();
    m_steppers->move(Steppers::Direction::Forward, right, deg2StepsSmall(target), !flag);
    // m_steppers->move(Steppers::Direction::Forward, right, deg2StepsSmall(target), accel);
    while(m_steppers->exec());
    delay(1000);
}

void Robot::StartToFirstLine()
{
    Serial.println("[[[[ StartToFirstLine ]]]]");
    //from StartArea:180CW to OnFirstLine:180CW
    partBackward(120 + ST_SAFETY_LARGE);
    partAdjust2(false);
    partBackward(250 + ST_SAFETY); // 120 = backend to wheelbase
    partAdjust2(true);
}

void Robot::FirstLineToSecondLine()
{
    Serial.println("[[[[ FirstLineToSecondLine ]]]]");
    //from OnFirstLine:180CW to OnSecondLine:180CW
    partBackward(650 + ST_SAFETY_LARGE);
    partAdjust2(true);
}

void Robot::SecondLineToThirdLine()
{
    Serial.println("[[[[ SecondLineToThirdLine ]]]]");
    //from OnSecondLine:180CW to OnThirdLine:270CW
    partBackward(640); //650
    partTurnRight(90);
    partForward(250 - ST_SAFETY_LARGE);
    partAdjust2(false);
    partBackward(250 + 300 + ST_SAFETY);
    partAdjust2(true);
}

bool Robot::ThirdLineToBallLine()
{
    Serial.println("[[[[ ThirdLineToBallLine ]]]]");
    //from OnThirdLine:270CW to OFF BallLine:180CW
    bool ret = false;
    partBackward(280 + 460);
    partTurnLeft(90);
    partForward(150); //part of 250 as non-ball zone

    int deg = HeadingSearch(45, -45); //narrow
    if (deg != BS_FAIL) {
        ret = HeadingToGetAndRecovery(deg);
    }
    partAdjust2(false);
    return ret;
}

void Robot::BallLineToThirdLine()
{
    Serial.println("[[[ BallLineToThirdLine ]]]]");
    //from OFF BallLine:180CW to OnThirdLine:90CW
    partBackward(250 - ST_LS2WHEELBASE);
    partTurnLeft(90);
    partBackward(280 + 460 + ST_SAFETY_LARGE);
    partAdjust2(true);
}

void Robot::ThirdLineToSecondLine()
{
    Serial.println("[[[ ThirdLineToSecondLine ]]]]");
    //from OnThirdLine:90CW to OnSecondLine:180CW
    partBackward(300);
    partTurnRight(90);
    partForward(650 - ST_SAFETY_LARGE);
    partAdjust2(true);
}

void Robot::SecondLineToFirstLine()
{
    Serial.println("[[[ SecondLineToFirstLine ]]]]");
    //from OnSecondLine:180CW to OnFirstLine:180CW
    partForward(650 - ST_SAFETY_LARGE);
    partAdjust2(true);
}

void Robot::FirstSecondGoal()
{
    Serial.println("[[[ FirstSecondGoal ]]]]");
    //from/to OnFirst/SecondLine:180CW
    partTurnRight(90);
    partForward(250 - ST_SAFETY_LARGE);
    partAdjust2(false);

    controlArm(false);
    controlFan(false);
    delay(2000);

    partBackward(250 - ST_LS2WHEELBASE);
    partTurnLeft(90);
    partBackward(ST_SAFETY_LARGE);
    partAdjust2(true);
}

void Robot::ThirdGoal()
{
    Serial.println("[[[ ThirdGoal ]]]]");
    //from OnThirdLine:90CW to OnThirdLine:270CW
    partBackward(300);
    partTurnRight(180);
    partForward(250 - ST_SAFETY_LARGE);
    partAdjust2(false);

    controlArm(false);
    controlFan(false);
    delay(2000);

    partBackward(250 + 300 + ST_SAFETY);
    delay(1000);
    partAdjust2(true);
}

void Robot::FirstLineToZeroLine()
{
    Serial.println("[[[ FirstLineToZeroLine ]]]]");
    //from OnFirstLine:180CW to ZeroLine:90CW
    partTurnLeft(90);
    partForward(300 + 280);
}

void Robot::ZeroLineToFirstLine()
{
    Serial.println("[[[ ZeroLineToFirstLine ]]]]");
    //from ZeroLine:90CW to OnFirstLine:180CW
    partBackward(300 + 280);
    partTurnRight(180);
    partForward(300 - ST_SAFETY_LARGE);
    partAdjust2(false);
    partBackward(250 - ST_LS2WHEELBASE);
    partTurnLeft(90);
    partBackward(ST_SAFETY_LARGE);
    partAdjust2(true);
}

bool Robot::TryToGet()
{
    Serial.println("[[[ TryToGet ]]]]");
    controlArm(false);
    controlFan(true);

    Serial.print("start");
    int counter = 0;
    for (; counter < BS_BallCheckCounterMax; counter++) {
        m_steppers->move(Steppers::Direction::Forward, Steppers::Direction::Forward,
                            mm2Steps(BS_BallCheckEveryMM), false);
            while(m_steppers->exec());
        // waitingBallCheck();
        if (isGetBall() == true) {
            controlArm(true);
            break;
        }
        Serial.print(".");
    }
    Serial.print("Dist:");
    Serial.println(counter, DEC);
    delay(1000);
    partBackward(BS_BallCheckEveryMM * counter);

    // waitingBallCheck();
    bool ret = isGetBall();
    controlFan(ret); //STOP Fan if fail
    controlArm(ret);

    return ret;
}

bool Robot::HeadingToGetAndRecovery(int degree)
{
    Serial.println("[[[ HeadingToGetAndRecovery ]]]]");
    //turn left is PLUS
    if (degree > 0) {
        partTurnLeft(degree);
    } else if (degree < 0) {
        partTurnRight(-degree);
    } else {
        Serial.println("degree = 0");
    }

    bool ret = TryToGet();

    if (degree > 0) {
        partTurnRight(degree);
    } else if (degree < 0) {
        partTurnLeft(-degree);
    } else {
        Serial.println("degree = 0");
    }
    
    return ret;
}

int Robot::HeadingSearch(int start, int end)
{
    Serial.print("[[[ HeadingSearch ]]]] :");
    Serial.print(start, DEC);
    Serial.print(", ");
    Serial.println(end, DEC);
    const int startDegree = start; // must > 0
    const int endDegree = end; // must < 0
    const int stepDegree = 2; // if =1, error is too large

    delay(1000);
    partTurnLeft(startDegree);
    delay(500);

    //start
    int target = -1;
    int degree = startDegree;
    uint16_t min = TOF_NONE;
    while (degree > endDegree) {
        degree -= stepDegree;
        m_steppers->move(Steppers::Direction::Forward, Steppers::Direction::Backward,
                            deg2StepsSmall(stepDegree), false);
        while(m_steppers->exec());
        if (m_tofFront < min) {
            min = m_tofFront;
            target = degree;
        }

        Serial.print(degree, DEC);
        Serial.print(": ");
        Serial.println(m_tofFront, DEC);
    }
    target -= 5; //adjust offset
    Serial.print("Target= ");
    Serial.println(target);

    delay(500);
    partTurnLeft(-endDegree);
    delay(1000);

    //check min
    if (min > 250) {
        return BS_FAIL;
    }
    return target;
}

bool Robot::SideSearchToGet(uint16_t sideLimit, int counterMax)
{
    Serial.println("[[[ SideSearchToGet ]]]]");
    bool ret = false;

    Serial.print("start");
    controlArm(false);
    controlFan(true);

    int counter = 0;
    bool isFoundAtFront = false;
    bool isFoundAtLeft = false;
    bool isFoundAtRight = false;

    int rightCounter = 0;
    int leftCounter = 0;
    uint16_t distance = 0;
    while (1) {
        m_steppers->move(Steppers::Direction::Forward, Steppers::Direction::Forward,
                            mm2Steps(BS_BallCheckEveryMM), false);
        while(m_steppers->exec());
        counter++;

        //check overrun
        if (counter > counterMax) {
            Serial.println("limit");
            break;
        }
        if (isGetBall() == true) {
            Serial.println("direct");
            break;
        }
        if (m_tofFront < 250) {
            Serial.print("front:");
            Serial.println(m_tofFront, DEC);
            distance = m_tofFront;
            isFoundAtFront = true;
            break;
        }
        if (m_tofLeft < TOF_LONG_LIMIT) {
            if (leftCounter++ > 7) {
                Serial.print("left:");
                Serial.println(m_tofLeft, DEC);
                distance = m_tofLeft;
                isFoundAtLeft = true;
                break;
            }
        }
        if (m_tofRight < sideLimit) {
            if (rightCounter++ > 7) {
                Serial.print("right:");
                Serial.println(m_tofRight, DEC);
                distance = m_tofRight;
                isFoundAtRight = true;
                break;
            }
        }
        Serial.print(".");
    }

    long steps = counter * mm2Steps(BS_BallCheckEveryMM);
    if (isFoundAtFront) {
        ret = TryToGet();
    } else {
        if (isFoundAtLeft || isFoundAtRight) {
            const int saftyLength = 120;
            const long saftySteps = mm2Steps(saftyLength);
            steps -= saftySteps;
            delay(1000);
            m_steppers->move(Steppers::Direction::Backward, Steppers::Direction::Backward,
                                saftySteps, true);
            while(m_steppers->exec());            
            int deg = 0;
            deg = (int)(atan2((double)distance, (double)saftyLength) * 180.0 / PI);
            deg += 5; //adjust offset
            if (isFoundAtRight) {
                deg = -deg;
            }
            Serial.print("heading:");
            Serial.println(deg, DEC);
            ret = HeadingToGetAndRecovery(deg);
        }
    }
    delay(1000);
    m_steppers->move(Steppers::Direction::Backward, Steppers::Direction::Backward,
                        steps, true);
    while(m_steppers->exec());
    delay(1000);

    if (isGetBall() == false) {
        ret = false;
        controlFan(false);
        controlArm(false);
    }
    return ret;
}

void Robot::exec()
{
#if 0
    //test - 360degree
    // partAdjust2(true);
    // partTurnLeft(90);
    // partTurnLeft(90);
    // partTurnLeft(90);
    // partTurnLeft(90);
    // partTurnRight(360);
    // partBackward(ST_SAFETY_LARGE);

    //test - arm
    controlFan(true);
    controlArm(false);
    while (1) {
        if (isGetBall() == true) {
            controlArm(true);
        } else {
            controlArm(false);
        }
        delay(1000);
    }

    //test - TryToGet
    // partAdjust2(false);
    // TryToGet();
    // partBackward(ST_SAFETY);

    //test - HeadingSearch
    // partAdjust2(false);
    // int ret = HeadingSearch();
    // if (ret != BS_FAIL) {
    //     HeadingToGetAndRecovery(ret);
    // }
    // partBackward(ST_SAFETY);

    // test - SideSearch
    // partAdjust2(false);
    // SideSearchToGet(TOF_LONG_LIMIT);
    // partBackward(ST_SAFETY);

    //test - Goal
    // partAdjust2(true);
    // controlFan(true);
    // while (isGetBall() == false) {
    //    // waitingBallCheck();
    // }
    // controlArm(true);
    // delay(2000);
    // FirstSecondGoal();
    // partBackward(ST_SAFETY_LARGE + 30);

    //test - START to FirstLine with no Search
    // StartToFirstLine();
    // FirstSecondGoal(); //push FreeBall
    // FirstLineToSecondLine();
    // SecondLineToThirdLine();
    // ThirdLineToBallLine();
    
    // BallLineToThirdLine();
    // ThirdLineToSecondLine();
    // SecondLineToFirstLine();
    // FirstLineToZeroLine();
    // ZeroLineToFirstLine();

#else
    //GAME
    StartToFirstLine();
    FirstSecondGoal(); //push FreeBall
    FirstLineToSecondLine();
    SecondLineToThirdLine();
    ThirdLineToBallLine();
    // partAdjust2(false);
    int selector = 0;
    int counter = 0;
    while (1) {
        //SIDE SEARCH
        if (isGetBall() ==  false) {
            selector++;
            if (selector < 16) {
                while (SideSearchToGet(TOF_LONG2_LIMIT, BS_SideBallCheckCounterMax_Long) == false);
            } else {
                //HEADING SEARCH
                if (counter > 0) {
                    Serial.print("### pre-search forward counter:");
                    Serial.println((counter + 1), DEC);
                    partForward(BS_SearchEveryMM * counter);            
                }
                for (; counter < BS_SearchCounterMax; counter++) {
                    int deg = HeadingSearch();
                    if (deg != BS_FAIL) {
                        if (HeadingToGetAndRecovery(deg) == true) {
                            break;
                        }
                    }
                    Serial.print("### forward counter =");
                    Serial.println((counter + 1), DEC);
                    partForward(BS_SearchEveryMM);
                }
                if (counter > 0) {
                    Serial.print("### post-search backward counter:");
                    Serial.println((counter + 1), DEC);
                    partBackward(BS_SearchEveryMM * counter);
                }
            }
            Serial.println("[[[ off BallLine ]]]");
            partBackward(ST_SAFETY);
            partAdjust2(false);
        }
        if (isGetBall() == false) {
            Serial.println("Fail to keep ball...(0)");
            continue;
        }

        Serial.println("[[[ Go to Goal ]]]");
        BallColor ballColor = checkBallColor();
        Serial.print("BallColor= ");
        Serial.println((int)ballColor, DEC);

        BallLineToThirdLine();
        // if (isGetBall() == true) {
        if (true) {
            //BallColor::Blue
            if (ballColor == BallColor::Blue) {
                ThirdGoal();
            } else {
                ThirdLineToSecondLine();
                // if (isGetBall() == true) {
                if (true) {
                    //BallColor::Yellow
                    if (ballColor == BallColor::Yellow) {
                        FirstSecondGoal();
                    } else {
                        //BallColor::Red or Unknown
                        SecondLineToFirstLine();
                        // if (isGetBall() == true) {
                        if (true) {
                            FirstSecondGoal();
                        } else {
                            Serial.println("Fail to keep ball...(3)");
                        }
                        FirstLineToSecondLine();
                    }
                } else {
                    Serial.println("Fail to keep ball...(2)");
                }
                SecondLineToThirdLine();
            }
        } else {
            Serial.println("Fail to keep ball...(1)");
        }
        ThirdLineToBallLine();
    }
#endif
}