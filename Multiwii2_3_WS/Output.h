#ifndef OUTPUT_H_
#define OUTPUT_H_

extern uint8_t PWM_PIN[8];

void initOutput();
void mixTable();
void writeServos();
void writeMotors(uint16_t batteryComp);

void blinkLED(uint8_t num, uint8_t ontime,uint8_t repeat);
#endif /* OUTPUT_H_ */
