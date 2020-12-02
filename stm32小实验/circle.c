
#include "circle.h"
#include <string.h>
#include "tim.h"
#include "can.h"
#include "motor.h"

#define LIMIT(IN,MIN,MAX)	(IN <= MIN ? MIN : (IN >= MAX ? MAX : IN))

circle_t circle={0};

void circle_pidinit(void){
  
    pid_init_absolute(&circle.M3508.Pidangle,0.01,0.008,0.001,9999);
//	  pid_init_absolute(&circle.M3508.pidspeed,0,0,0,9999);
}

void circle_angleinit(void){
     Circle_Continue(&circle.M3508.Mc,circle.M3508.Rx.Angle);
    circle.M3508.Tar.Angle = circle.M3508.Mc.Angle ;
	  circle.M3508.Tar.Circle=circle.M3508.Mc.Circle;
}
//void circle_LPFIn(void){
//	
//   circle.M3508.LPf.Angle=0.8 * circle.M3508.Rx.Angle + 0.2 * circle.M3508.LPf.Angle;
//}

void circle_GetMoveData(RemoteData_t RDMsg){
    circle.MoveData.Right      = RDMsg.Ch2;
    circle.MoveData.Front      = RDMsg.Ch3;
    circle.MoveData.ClockWise  = RDMsg.Ch0;
//    circle.M3508.TarSpeed = (int16_t)(-move.MoveData.Right / GAIN_I - move.MoveData.Front / GAIN_J + move.MoveData.ClockWise / GAIN_K);
	
    if(RDMsg.Ch0 > 500 || RDMsg.Ch0 < -500 ){
//			HAL_Delay(100);
			circle.M3508.Tar.Angle += 2048;
//      int a=1;
//			a++;
//		  circle.M3508.Tar.Angle-=circle.M3508.Mc.Angle;
//			circle.M3508.Tar.Circle-=circle.M3508.Mc.Circle;
		}
//		else {
//			    circle.M3508.Tar.Angle = circle.M3508.Mc.Angle;
//			    circle.M3508.Tar.Circle = circle.M3508.Mc.Circle;
////			circle.M3508.Tar.Angle = 0;
////			circle.M3508.Tar.Circle = 0;
//		}
			if(circle.M3508.Tar.Angle > 8000){
			circle.M3508.Tar.Circle++;
		  circle.M3508.Tar.Angle=0 ;
			}
			
					
}


void cirlce_PidRun(void)
{
	
	circle.M3508.Output=pid_absolute_update(circle.M3508.Tar.Angle+circle.M3508.Tar.Circle*8192,circle.M3508.Mc.Angle+circle.M3508.Mc.Circle*8192,&circle.M3508.Pidangle);
//	circle.M3508.TarSpeed=pid_absolute_update(circle.M3508.Tarangle,circle.M3508.LPf.Angle,&circle.M3508.Pidangle);
//	circle.M3508.Output=pid_absolute_update(circle.M3508.TarSpeed,circle.M3508.LPf.Speed,&circle.M3508.pidspeed);
	circle.M3508.Output=LIMIT(circle.M3508.Output,-3000,3000);
}

void circle_CanTransmit(void)
{
    if(Observer.Tx.DR16_Rate>15)
    {

            circle.CanData[6]=(uint8_t)(circle.M3508.Output>>8);
            circle.CanData[7]=(uint8_t)(circle.M3508.Output);

    }   
    else
    {
        memset(circle.CanData,0,sizeof(circle.CanData));
    }
    CAN2_Transmit(0x200,circle.CanData);
}

void circle_Process(RemoteData_t RDMsg)
{
    circle_GetMoveData(RDMsg);
    cirlce_PidRun();
    circle_CanTransmit();
}


