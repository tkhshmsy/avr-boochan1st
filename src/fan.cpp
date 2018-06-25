#include "fan.h"

#define FAN_ON (1)
#define FAN_OFF (0)
#define ARM_OPEN (90)
#define ARM_CLOSE (10)

Fan::Fan(int port)
    : m_port(port)
    , m_servo(NULL)
    , m_isActive(false)
    , m_isClosed(false)
{
    pinMode(m_port, OUTPUT);
    digitalWrite(m_port, FAN_OFF);
}

void Fan::setArmServo(Servo *servo)
{
    m_servo = servo;
}

void Fan::setActive(bool flag)
{
    m_isActive = flag;
    if (m_isActive == true) {
        Serial.println("*** FAN ON ***");
        digitalWrite(m_port, FAN_ON);
    } else {
        Serial.println("*** FAN OFF ***");
        digitalWrite(m_port, FAN_OFF);
    }
}

void Fan::setClose(bool flag) {
    m_isClosed = flag;
    if (m_isClosed == true) {
        Serial.println("*** ARM CLOSE ***");
        if (m_servo != NULL) {
            m_servo->write(ARM_CLOSE);
        }
    } else {
        Serial.println("*** ARM OPEN ***");
        if (m_servo != NULL) {
            m_servo->write(ARM_OPEN);
        }
    }
}

bool Fan::active()
{
    return m_isActive;
}

bool Fan::closed()
{
    return m_isClosed;
}