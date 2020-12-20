/* includes ------------------------------------------------------------------*/
#include "chassis.h"
#include <string.h>
#include "tim.h"
#include "can.h"
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
#define GAIN_I 0.1
#define GAIN_J 0.1
#define GAIN_K 0.2
/* variables -----------------------------------------------------------------*/
Chassis_t Chassis = {0};
uint8_t i=3;
/* function ------------------------------------------------------------------*/
void  Chassis_PidInit(void)
{
     pid_init_increment(&Chassis.M3508[i].PidSpeed,4,0.1,0.001,9999);
    
     pid_init_increment(&Chassis.M3508[i].PidCurrent,0.8,0.1,0,9999);   
}

void Chassis_LPfIn(void)
{
     Chassis.M3508[i].LPf.Speed = 0.8 * Chassis.M3508[i].Rx.Speed + 0.2 * Chassis.M3508[i].LPf.Speed;

     Chassis.M3508[i].LPf.Current = 0.8 * Chassis.M3508[i].Rx.Current + 0.2 * Chassis.M3508[i].LPf.Current;
 
}

void Chassis_GetMoveData(RemoteData_t RDMsg)
{
    Chassis.MoveData.Right      = RDMsg.Ch2;
    Chassis.MoveData.Front      = RDMsg.Ch3;
    Chassis.MoveData.ClockWise  = RDMsg.Ch0;

//    Chassis.M3508[0].TarSpeed = (int16_t)( Chassis.MoveData.Right / GAIN_I + Chassis.MoveData.Front / GAIN_J + Chassis.MoveData.ClockWise / GAIN_K);
//    Chassis.M3508[1].TarSpeed = (int16_t)( Chassis.MoveData.Right / GAIN_I - Chassis.MoveData.Front / GAIN_J + Chassis.MoveData.ClockWise / GAIN_K);
//    Chassis.M3508[2].TarSpeed = (int16_t)(-Chassis.MoveData.Right / GAIN_I + Chassis.MoveData.Front / GAIN_J + Chassis.MoveData.ClockWise / GAIN_K);
    Chassis.M3508[3].TarSpeed = (int16_t)(-Chassis.MoveData.Right / GAIN_I - Chassis.MoveData.Front / GAIN_J + Chassis.MoveData.ClockWise / GAIN_K);
}

void Chassis_LPfOut(void)
{
    Chassis.M3508[i].OutputLpf = 0.8 * Chassis.M3508[i].Output + 0.2 * Chassis.M3508[i].OutputLpf;
}


void Chassis_PidRun(void)
{
    Chassis.M3508[i].TarCurrent = pid_increment_update(Chassis.M3508[i].TarSpeed, Chassis.M3508[i].LPf.Speed, &Chassis.M3508[i].PidSpeed);

    Chassis.M3508[i].Output = pid_increment_update(Chassis.M3508[i].TarCurrent, Chassis.M3508[i].LPf.Current, &Chassis.M3508[i].PidCurrent);

   
}

void Chassis_CanTransmit(void)
{
    if(Observer.Tx.DR16_Rate>15)
    {
            Chassis.CanData[2*i]=(uint8_t)(Chassis.M3508[i].OutputLpf>>8);
            Chassis.CanData[2*i+1]=(uint8_t)(Chassis.M3508[i].OutputLpf);
    }   
    else
    {
        memset(Chassis.CanData,0,sizeof(Chassis.CanData));
    }
    CAN2_Transmit(0x200,Chassis.CanData);
}

void Chassis_Process(RemoteData_t RDMsg)
{
    Chassis_GetMoveData(RDMsg);
    Chassis_LPfIn();
    Chassis_PidRun();
    Chassis_LPfOut();
    Chassis_CanTransmit();
}

/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
