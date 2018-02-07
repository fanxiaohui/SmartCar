#include "PID.h"

void PID_init() 
{
	pid.SetSpeed = 0.0;
	pid.ActualSpeed = 0.0;
	pid.err = 0.0;
	pid.err_last = 0.0;
	pid.err_next = 0.0;
	pid.Kp = 0.0;
	pid.Ki = 0.0;
	pid.Kd = 0.0;
}
