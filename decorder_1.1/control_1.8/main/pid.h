void PIDController_Init(PIDController *pid);
void update_PID_gain(void);
void PIDController_Update(PIDController *pid, float setpoint, int measurement, uint8_t i);
void run_position_loop();
void pid_app_main(void);