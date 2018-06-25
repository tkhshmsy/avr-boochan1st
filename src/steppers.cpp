#include "steppers.h"

const unsigned int Steppers::m_speedTable[] = {
    //unit: microseconds
    6068, //80mm/sec
    4821, 3991, 3405, 2969, 2632,
    2363, 2145, 1963, 1810, 1679,
    1565, 1466, 1379, 1301, 1232,
    1170, 1114, 1063, 1016, 973 //500mm/sec

    // 10000,
    // 4869,
    // 3358, 2563, 2072, 1739, 1498,
    // 1316, 1173, 1059, 964, 885,
    // 818, 761, 711, 667, 628,
    // 594, 563, 535, 510, 486
};
const unsigned int Steppers::m_speedSpan = 10; //default 5
const unsigned int Steppers::m_speedMax = 12; //default 22

Steppers::Steppers(int ls, int ld, int rs, int rd)
    : m_portLeftDirection(ld)
    , m_portRightDirection(rd)
    , m_portLeftStep(ls)
    , m_portRightStep(rs)
    , m_isActive(false)
    , m_isAccelMode(false)
    , m_leftDirection(0)
    , m_rightDirection(0)
    , m_leftCounter(0)
    , m_rightCounter(0)
    , m_targetSteps(0)
    , m_currentSteps(0)
    , m_currentSpeed(0)
    , m_localStep(-1)
{
    pinMode(m_portLeftDirection, OUTPUT);
    pinMode(m_portRightDirection, OUTPUT);
    pinMode(m_portLeftStep, OUTPUT);
    pinMode(m_portRightStep, OUTPUT);
}

void Steppers::leftStepOutput()
{
    static bool output = false;
    static int8_t direction = 0;
    if (direction != m_leftDirection) {
        if (m_leftDirection >= 0) {
            digitalWrite(m_portLeftDirection, LOW);
        } else {
            digitalWrite(m_portLeftDirection, HIGH);
        }
        direction = m_leftDirection;
    }
    digitalWrite(m_portLeftStep, output);
    output = !output;
}

void Steppers::rightStepOutput()
{
    static bool output = false;
    static int8_t direction = 0;
    if (direction != m_rightDirection) {
        if (m_rightDirection >= 0) {
            digitalWrite(m_portRightDirection, LOW);
        } else {
            digitalWrite(m_portRightDirection, HIGH);
        }
        direction = m_rightDirection;
    }
    digitalWrite(m_portRightStep, output);
    output = !output;
}

bool Steppers::exec()
{
    // Serial.println("e");
    if (m_isActive == false) {
        return m_isActive;
    }

    if (m_isAccelMode == true) {
        if (++m_localStep >= m_speedSpan) {
            if (m_currentSteps < (m_targetSteps / 2)) {
                if (m_currentSpeed < (m_currentSpeedMax - 1)) {
                    m_currentSpeed++;
                }
            } else {
                if ((m_targetSteps - m_currentSteps) < (m_speedSpan * m_currentSpeedMax)) {
                    if (m_currentSpeed > 0) {
                        m_currentSpeed--;
                    }
                }
            }
            m_localStep = 0;
        }
    } else {
        m_currentSpeed = 0;
    }

    delayMicroseconds(m_speedTable[m_currentSpeed]);

    if (m_leftDirection != 0) {
        m_leftCounter += m_leftDirection;
        leftStepOutput();
    }
    if (m_rightDirection != 0) {
        m_rightCounter += m_rightDirection;
        rightStepOutput();
    }
    m_currentSteps++;
    if (m_currentSteps >= m_targetSteps) {
        m_isActive = false;
        m_localStep = -1; //reset
    }

    return m_isActive;
}

bool Steppers::isAccelMoving()
{
    if (m_isAccelMode) {
        if (m_isActive) {
            return true;
        }
    }
    return false;
}

void Steppers::forceStop()
{
    if (m_isAccelMode == true) {
        m_targetSteps = m_currentSteps + (m_currentSpeed * m_speedSpan);
        m_localStep = m_speedSpan + 1;
    } else {
        m_currentSpeed = 0;
        m_isActive = false;
    }
}

void Steppers::move(Direction left, Direction right, long steps, bool accel,unsigned int max)
{
    if (m_isActive == true) {
        return;
    }

    if (left == Direction::Forward) {
        m_leftDirection = 1;
    } else if (left == Direction::Backward) {
        m_leftDirection = -1;
    } else {
        m_leftDirection = 0;
    }
    if (right == Direction::Forward) {
        m_rightDirection = 1;
    } else if (right == Direction::Backward) {
        m_rightDirection = -1;
    } else {
        m_rightDirection = 0;
    }

    m_currentSpeedMax = max;
    if (max == 0 || max >= m_speedMax) {
        m_currentSpeedMax = m_speedMax;
    }

    m_targetSteps = steps;
    m_isAccelMode = accel;

    m_localStep = -1;
    m_currentSteps = 0;
    m_currentSpeed = 0;
    m_isActive = true;
}

long Steppers::steps() const
{
    return m_currentSteps;
}

void Steppers::resetCounter()
{
    m_leftCounter = 0;
    m_rightCounter = 0;
}

long Steppers::leftCounter() const
{
    return m_leftCounter;
}

long Steppers::rightCounter() const
{
    return m_rightCounter;
}