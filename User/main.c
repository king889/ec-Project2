#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "LED.h"
#include "Timer.h"
#include "Key.h"
#include "RP.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include "PID.h"
#include <stdio.h>
#include "string.h"
#include "stdlib.h"

uint16_t RP1, RP2, RP3, RP4 = 0;
uint8_t RxData;
uint8_t State=1;
int16_t Speed1,Speed2;        // 改为有符号整型
int32_t Location1,Location2;     // 改为有符号长整型
uint8_t Keynum;

// PID参数定义在外面
PID_t Motor1_PID = {
    .Target = 0,
    .Actual = 0,
    .Out = 0,
    .Kp = 0.35,
    .Ki = 0.02, 
    .Kd = 0.4,
    .Error0 = 0,
    .Error1 = 0,
    .Error2 = 0,
    .OutMax = 100,
    .DeadZone =0,
    .OutMin = -100
};

PID_t Motor2_PID_Speed = {
    .Target = 0,
    .Actual = 0,
    .Out = 0,
    .Kp = 0.35,
    .Ki = 0.02, 
    .Kd = 0.4,
    .Error0 = 0,
    .Error1 = 0,
    .Error2 = 0,
    .DeadZone =0,
    .OutMax = 100,
    .OutMin = -100
};

PID_t Motor2_PID_Pos = {
    .Target = 0,
    .Actual = 0,
    .Out = 0,
    .Kp = 1,
    .Ki = 0.08, 
    .Kd = 1,
    .Error0 = 0,
    .Error1 = 0,
    .DeadZone =5,
    .Error2 = 0,
    .OutMax = 74,
    .OutMin = -74
};



int main(void)
{
    OLED_Init();
    RP_Init();
    Serial_Init();

    Encoder_Init();
    Motor_Init();
    Key_Init();

    Timer_Init(); 

    while (1)
    {
		Keynum=Key_GetNum();
        if (Keynum==1)
        {
            State = (State % 2) + 1;
            OLED_Clear();
            OLED_Update();
            Serial_Printf("OK");
            Location1=0;Location2=0;
            Motor_SetPWM1(0);

        }        
        if (State==1)
        {
            if (Serial_RxFlag == 1)
            {
                if (strncmp(Serial_RxPacket, "speed%", 6) == 0)
                {
                    Motor1_PID.Target = atoi(Serial_RxPacket + 6);
                }
                Serial_RxFlag = 0;
            }		
            OLED_Printf(0,0,OLED_8X16,"Speed_Control");
            OLED_Printf(0, 16, OLED_8X16, "Sp1:%+06d", Speed1);
            OLED_Printf(0, 32, OLED_8X16, "Tar:%+08ld", Motor1_PID.Target);
            OLED_Update();
        }                                
        else if (State==2)
        {
            OLED_Clear();
            OLED_Printf(0,0,OLED_8X16,"Pos_Control");
            OLED_Printf(0, 16, OLED_8X16, "Pos1:%+06d", Location1);
            OLED_Printf(0, 32, OLED_8X16, "Pos2:%+06d", Location2);
            OLED_Update();


        }
        
        
        
        Serial_Printf("%d,%f,%d,%f,%d,%d\r\n",Speed1,Motor1_PID.Target,Speed2,Motor2_PID_Speed.Target,Location1,Location2);
    }
}

void TIM1_UP_IRQHandler(void)
{
    static uint16_t Count1,Count2;

    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        Key_Tick();
        

        
        if(State==1)
        {
            Count1++;
            if (Count1 >= 10)
            {
                Speed1 = Encoder_Get1();
                Location1 += Speed1;
                Speed2=Encoder_Get2();
                Location2+=Speed2;
                Count1 = 0;

                
                // PID控制
                Motor1_PID.Actual = Speed1;        // 更新实际值
                PID_Update(&Motor1_PID);           // 更新PID计算
                Motor_SetPWM1(-(int8_t)Motor1_PID.Out); // 设置电机PWM
            }
        }
        else if (State==2)
        {
            Count2++;
            if (Count2 >= 10)
            {
                Count2 = 0;
                Speed1 = Encoder_Get1();
                Location1 += Speed1;
                Speed2=Encoder_Get2();
                Location2+=Speed2;
                
                Motor2_PID_Pos.Target = Location1;
                Motor2_PID_Pos.Actual = Location2;
                PID_Update(&Motor2_PID_Pos);

                Motor2_PID_Speed.Target = Motor2_PID_Pos.Out;
                Motor2_PID_Speed.Actual = Speed2;        // 更新实际值
                PID_Update(&Motor2_PID_Speed);           // 更新PID计算
                Motor_SetPWM2(Motor2_PID_Speed.Out); // 设置电机PWM
            }
        }           
        }
        
        
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
}


