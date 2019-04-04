#include "shoot.h"

PidTypeDef TR_Speedloop;

void TriggerMotor_PID_Config(void)
{
    static const fp32 Trigger_SpeedPID[3]={85,0.0,10};
    //³õÊ¼»¯PID
    PID_Init(&TR_Speedloop,PID_POSITION,Trigger_SpeedPID,10000,5000);

}
void TriggerMotor_Ctrl(RC_ctrl_t *Rc)
{
    TR_Speedloop.set = +(Rc->rc.s[0] == 1) ? 130 : 0;
    //TR_Speedloop.set = (Rc->mouse.press_l == 1) ? 80 : 0;
    TR_Speedloop.out=PID_Calc(&TR_Speedloop,TREncoder.filter_rate,TR_Speedloop.set);
}
void TriggerMotor_Out(void)
{
	Set_CloudMotor_Current(CloudMotor_Out()[0],CloudMotor_Out()[1],TR_Speedloop.out);
}
