//battery_motor.h
unsigned char battery_init(void);
unsigned char battery_alert_init(void);
unsigned int battery_voltage_get(void);
void battery_itc2942_set(void);
void motor_pwm_init(void);

