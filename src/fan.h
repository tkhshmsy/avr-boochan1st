#ifndef __FAN_H__
#define __FAN_H__

#include <Arduino.h>
#include <Servo.h>

class Fan
{
public:
    Fan(int port);
    void setArmServo(Servo *servo);

    void setActive(bool flag);
    bool active();
    
    void setClose(bool flag);
    bool closed();

private:
    int m_port;

    Servo *m_servo;
    bool m_isActive;
    bool m_isClosed;
};

#endif