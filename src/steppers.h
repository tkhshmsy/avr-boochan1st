#ifndef __STEPPERS_H__
#define __STEPPERS_H__

#include <Arduino.h>

class Steppers
{
public:
    enum Direction {
        Forward = 1,
        Stop = 0,
        Backward = -1,
    };

    Steppers(int ls, int ld, int rs, int rd);

    bool exec(); //call from Loop()
    bool isAccelMoving(); //for sensing-control

    // -1=back, 0=stop, 1=forward
    void move(Direction left, Direction right, long steps, bool accel  = false, unsigned int max = 0);
    void forceStop();
    long steps() const;

    void resetCounter();
    long leftCounter() const;
    long rightCounter() const;

private:
    void leftStepOutput();
    void rightStepOutput();

    const int m_portLeftDirection;
    const int m_portRightDirection;
    const int m_portLeftStep;
    const int m_portRightStep;

    bool m_isActive;
    bool m_isAccelMode;

    int8_t m_leftDirection;
    int8_t m_rightDirection;
    long m_leftCounter;
    long m_rightCounter;

    long m_targetSteps;
    long m_currentSteps;

    unsigned int m_currentSpeed;
    unsigned int m_currentSpeedMax;
    unsigned int m_localStep;
    static const unsigned int m_speedSpan;
    static const unsigned int m_speedMax;
    static const unsigned int m_speedTable[];
};

#endif //__STEPPERS_H__
