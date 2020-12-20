
circle_t circle={0};
uint16_t j=1;
float diff;
int16_t tar,rx;
void circle_pidinit(void){
    pid_init_absolute(&circle.M3508.Pidangle,0.029,0.018,0.4,9999);
	  pid_init_absolute(&circle.M3508.pidspeed,0.23,0.13,0.54,9999);  /*速度环控制*/
}

void circle_angleinit(void){
     Circle_Continue(&circle.M3508.Mc,circle.M3508.Rx.Angle);
    circle.M3508.Tar.Angle = circle.M3508.Mc.Angle ;
	  circle.M3508.Tar.Circle = circle.M3508.Mc.Circle;
}



void circle_GetMoveData(RemoteData_t RDMsg){

    if((RDMsg.S1 ==1) && (j==1)){
			circle.M3508.Tar.Angle += 5424;
			circle.M3508.Tar.Circle += 3;
      j=0;
		}
		if((RDMsg.S1 == 3) && (j==0)){
		j=1;
		}
		if(circle.M3508.Tar.Angle > 8192)
			{
			circle.M3508.Tar.Circle++;
		  circle.M3508.Tar.Angle=circle.M3508.Tar.Angle-8192 ;
			}		
}


void cirlce_PidRun(void)
{
	
//	diff=819 * (float)(circle.M3508.Tar.Circle + (float)circle.M3508.Tar.Angle/ 8192) - angle;
	tar = (circle.M3508.Tar.Circle + circle.M3508.Tar.Angle/8192)*819;
	rx = (circle.M3508.Mc.Circle + circle.M3508.Mc.Angle/8192)*819;
	diff=circle.M3508.Tar.Circle*8192 + circle.M3508.Tar.Angle - circle.M3508.Mc.Angle-circle.M3508.Mc.Circle*8192;
	circle.M3508.TarSpeed=pid_absolute_update(diff,0,&circle.M3508.Pidangle);
  circle.M3508.Output=pid_absolute_update(circle.M3508.TarSpeed,circle.M3508.Rx.Speed,&circle.M3508.pidspeed);
	circle.M3508.Output=LIMIT(circle.M3508.TarSpeed,-3000,3000);
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



