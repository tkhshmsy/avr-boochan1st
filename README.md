# avr-boochan1st
Boo-Chan-1st is a robot for inrof2018 using arduino

## H/W
- arduino nano
    - i2c
        - TCA9548A i2c-multiplex
        - 3x VL53L0X ToF-range-sensor
    - S/W i2c
        - TCS34725 color-sensor
    - 2x A9488 bi-polar-stepping-motor-driver
    - 2x S4283-51 photo-ic
    - RCservo
    - DCmotor

## depends
- platformio
    - SoftwareWire (ID832)
    - TI TCA9548A (ID2087)
    - VL53L0X (ID854)
    - FlexiTimer2 (ID138)

## author
- tkhshmsy@gmail.com
    - https://github.com/tkhshmsy/avr-boochan1st

## license
- MIT
