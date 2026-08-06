/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：5.7
修改时间：2021-04-29


Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: 5.7
Update：2021-04-29

All rights reserved
***********************************************/
#include "control.h"
#include "IR_Module.h"
#include "imu/imu.h"
#include "ball_balance.h"
#include "ball_hold_lap.h"
#include "ball_state_observer.h"
#include "ball_static_task.h"
#include "ball_turn_feedforward.h"
#include "servo.h"
#include "straight_turn_test.h"
#include "turn_calibration.h"
#include "uart_callback.h"

u8 CCD_count,ELE_count;
int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor;
int Servo=SERVO_NEUTRAL_PULSE_US;
Encoder OriginalEncoder; 					//编码器原始数据   
Motor_parameter MotorA,MotorB;				//左右电机相关变量
float Velocity_KP=400,Velocity_KI=300;	
int Run_Mode=RUN_MODE_MENU_DEFAULT;//小车运行模式
u8 Flag_Stop=1;//小车启动标志位
volatile uint8_t Menu_Active=1U;
volatile uint8_t Menu_Selection=RUN_MODE_MENU_DEFAULT;
volatile uint8_t Menu_SelectionIndex;
const menu_item_t Menu_Items[MENU_MODE_COUNT] =
{
	{RUN_MODE_STRAIGHT_TURN, "ONE LAP"},
	{RUN_MODE_BALL_LAP, "BALL LAP"},
	{RUN_MODE_DRIBBLE, "DRIBBLE"},
	{RUN_MODE_BALL_HOLD_LAP, "BALL HOLD"},
	{RUN_MODE_BALL_STATIC, "STATIC HYB"},
	{RUN_MODE_SERVO_ADJUST, "SERVO ADJ"},
	{RUN_MODE_IMU_DEBUG, "IMU DEBUG"}
};
static u8 Reset_Left_PI, Reset_Right_PI;
static ball_turn_feedforward_t ball_turn_feedforward_state;
volatile uint8_t ball_turn_feedforward_entry_active;

static void update_ball_state_observer(uint32_t now_ms)
{
    ball_vision_measurement_t measurement;
    uint32_t frame_before;
    uint32_t frame_after;

    do
    {
        frame_before = vision_ball_frame_count;
        measurement.position_mm = vision_ball_position_mm;
        measurement.velocity_mm_s = vision_ball_velocity_mm_s;
        measurement.sample_ms = vision_ball_last_update_ms;
        measurement.valid = vision_ball_position_valid;
        frame_after = vision_ball_frame_count;
    } while (frame_before != frame_after);

    measurement.frame_count = frame_after;
    ball_state_observer_update(
        &ball_state_observer,
        &measurement,
        now_ms);
}

static void update_ball_turn_feedforward(void)
{
	uint8_t controller_context_active =
		(Menu_Active == 0U &&
		 Run_Mode == RUN_MODE_BALL_HOLD_LAP &&
		 ball_hold_lap_controller_enabled() != 0U) ? 1U : 0U;
	uint8_t arc_active =
		(StraightTurnState == STRAIGHT_TURN_ARC_1 ||
		 StraightTurnState == STRAIGHT_TURN_ARC_2) ? 1U : 0U;

	if (controller_context_active != 0U)
	{
		if (ball_turn_feedforward_update(
				&ball_turn_feedforward_state,
				arc_active,
				StraightTurnCommandSpeed,
				StraightTurnCommandOmegaRadS) == 0U)
		{
			ball_turn_feedforward_reset(
				&ball_turn_feedforward_state);
		}
	}
	else
	{
		ball_turn_feedforward_reset(
			&ball_turn_feedforward_state);
	}

	ball_turn_feedforward_entry_active =
		ball_turn_feedforward_state.entry_active;
	ball_balance_set_turn_feedforward(
		ball_turn_feedforward_state.output_us);
}

static void menu_select_mode(uint8_t mode)
{
	uint8_t index;

	for (index = 0U; index < MENU_MODE_COUNT; index++)
	{
		if (Menu_Items[index].mode == mode)
		{
			Menu_SelectionIndex = index;
			Menu_Selection = mode;
			return;
		}
	}

	Menu_SelectionIndex = 0U;
	Menu_Selection = Menu_Items[0].mode;
}

void TIMER_0_INST_IRQHandler(void)
{
    static int lastRunMode = -1;

    if(DL_TimerA_getPendingInterrupt(TIMER_0_INST))
    {
        if(DL_TIMER_IIDX_ZERO)
        {
            tick_ms += 5U;
			
			Key();
			LED_Flash(100);
			Get_Velocity_From_Encoder(Get_Encoder_countA,Get_Encoder_countB);
			Get_Encoder_countA=Get_Encoder_countB=0;
			update_ball_state_observer((uint32_t)tick_ms);
			if (Run_Mode != lastRunMode)
			{
				Reset_Velocity_PI();
				IR_Differential_OneLap_Reset();
				TurnCalibration_Reset();
				StraightTurnTest_Reset();
				ball_static_task_reset();
				ball_hold_lap_reset();
				lastRunMode = Run_Mode;
			}
			if (Menu_Active == 0U &&
			    Run_Mode == RUN_MODE_BALL_HOLD_LAP)
			{
				ball_hold_lap_update();
			}
			if (Menu_Active == 0U &&
			    Run_Mode == RUN_MODE_BALL_STATIC)
			{
				ball_static_task_update();
			}
			if (Menu_Active == 0U &&
			    Run_Mode == RUN_MODE_BALL_STATIC)
			{
				ball_balance_set_enabled(
					ball_static_task_controller_enabled());
			}
			else if (Menu_Active == 0U &&
			         Run_Mode == RUN_MODE_BALL_HOLD_LAP)
			{
				ball_balance_set_enabled(
					ball_hold_lap_controller_enabled());
			}
			else
			{
				ball_balance_set_enabled(
					(Menu_Active == 0U) &&
					(Run_Mode == RUN_MODE_BALL_LAP ||
					 Run_Mode == RUN_MODE_DRIBBLE));
			}
			ball_balance_set_predictive_guard_enabled(
				(Menu_Active == 0U &&
				 Run_Mode == RUN_MODE_BALL_HOLD_LAP &&
				 ball_hold_lap_controller_enabled() != 0U) ?
					1U : 0U);
			update_ball_turn_feedforward();
			ball_balance_set_vehicle_acceleration(
				(Menu_Active == 0U &&
				 (Run_Mode == RUN_MODE_BALL_LAP ||
				  Run_Mode == RUN_MODE_DRIBBLE ||
				  Run_Mode == RUN_MODE_BALL_HOLD_LAP) &&
				 Flag_Stop == 0U) ?
					StraightTurnStartupAccelerationMps2 : 0.0f);
			if (Menu_Active != 0U)
			{
				control_uart_set_mode(CONTROL_UART_DISABLED);
			}
			else if (Run_Mode == RUN_MODE_SERVO_ADJUST)
			{
				control_uart_set_mode(CONTROL_UART_SERVO_ADJUST);
			}
			else if (Run_Mode == RUN_MODE_BALL_STATIC)
			{
				control_uart_set_mode(
					CONTROL_UART_OPEN_LOOP_TUNING);
			}
			else if (Run_Mode == RUN_MODE_BALL_LAP ||
			         Run_Mode == RUN_MODE_DRIBBLE ||
			         Run_Mode == RUN_MODE_BALL_HOLD_LAP)
			{
				control_uart_set_mode(CONTROL_UART_PID_TUNING);
			}
			else
			{
				control_uart_set_mode(CONTROL_UART_DISABLED);
			}
			ball_balance_update();
			Servo = (int)servo_get_pulse_us();
			if (Run_Mode == RUN_MODE_BALL_STATIC)
			{
				Flag_Stop = 1;
				Reset_Velocity_PI();
				MotorA.Target_Encoder = 0.0f;
				MotorB.Target_Encoder = 0.0f;
				motor_set_pwm(0,0);
				return;
			}
			if (Flag_Stop)
			{
				Reset_Velocity_PI();
				MotorA.Target_Encoder = 0.0f;
				MotorB.Target_Encoder = 0.0f;
				motor_set_pwm(0,0);
				return;
			}
			if(Run_Mode==RUN_MODE_ONE_LAP){
				IR_Differential_OneLap();
			}else if(Run_Mode==RUN_MODE_TURN_CAL){
				TurnCalibration_Run();
			}else if(Run_Mode==RUN_MODE_STRAIGHT_TURN ||
			         Run_Mode==RUN_MODE_BALL_LAP ||
			         Run_Mode==RUN_MODE_DRIBBLE ||
			         Run_Mode==RUN_MODE_BALL_HOLD_LAP){
				StraightTurnTest_Run();
			}
			if (Flag_Stop)
			{
				Reset_Velocity_PI();
				motor_set_pwm(0,0);
				return;
			}
//			//计算左右电机对应的PWM
			MotorA.Motor_Pwm = Incremental_PI_Left(MotorA.Current_Encoder,MotorA.Target_Encoder);	
			MotorB.Motor_Pwm = Incremental_PI_Right(MotorB.Current_Encoder,MotorB.Target_Encoder);
			Limit_Pwm(7500) ;
			motor_set_pwm(MotorA.Motor_Pwm,MotorB.Motor_Pwm);
		}
    }
}

/**************************************************************************
Function: Get_Velocity_From_Encoder
Input   : none
Output  : none
函数功能：读取编码器和转换成速度
入口参数: 无 
返回  值：无
**************************************************************************/	 	
void Get_Velocity_From_Encoder(int Encoder1,int Encoder2)
{
	
	//Retrieves the original data of the encoder
	//获取编码器的原始数据
	float Encoder_A_pr,Encoder_B_pr; 
	OriginalEncoder.A=-Encoder1;	
	OriginalEncoder.B=-Encoder2;	
	Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B;
	//编码器原始数据转换为车轮速度，单位m/s
	MotorA.Current_Encoder =
		Encoder_A_pr * Frequency / ENCODER_LEFT_COUNTS_PER_METER;
	MotorB.Current_Encoder =
		Encoder_B_pr * Frequency / ENCODER_RIGHT_COUNTS_PER_METER;
	
}
//差速运动学逆解：Vx单位m/s，Vz单位rad/s，正值为左转
void Get_Target_Encoder(float Vx,float Vz)
{
	float amplitude=3.5f; //Wheel target speed limit //车轮目标速度限幅

	MotorA.Target_Encoder =
		Vx - 0.5f * DRIVE_WHEEL_SPACING * Vz;
	MotorB.Target_Encoder =
		Vx + 0.5f * DRIVE_WHEEL_SPACING * Vz;
	MotorA.Target_Encoder=target_limit_float(MotorA.Target_Encoder,-amplitude,amplitude); 
	MotorB.Target_Encoder=target_limit_float(MotorB.Target_Encoder,-amplitude,amplitude); 
}


/**************************************************************************
Function: Absolute value function
Input   : a：Number to be converted
Output  : unsigned int
函数功能：绝对值函数
入口参数：a：需要计算绝对值的数
返回  值：无符号整型
**************************************************************************/
int myabs(int a)
{
	int temp;
	if(a<0)  temp=-a;
	else temp=a;
	return temp;
}

int Turn_Off(void)
{
	u8 temp = 0;
//	if(Voltage>700&&EN==0)//电压高于7V且使能开关打开
//	{
//		temp = 1;
//	}
	return temp;			
}
/**************************************************************************
Function: PWM_Limit
Input   : IN;max;min
Output  : OUT
函数功能：限制PWM赋值
入口参数: IN：输入参数  max：限幅最大值  min：限幅最小值 
返回  值：限幅后的值
**************************************************************************/	 	
float PWM_Limit(float IN,float max,float min)
{
	float OUT = IN;
	if(OUT>max) OUT = max;
	if(OUT<min) OUT = min;
	return OUT;
}
/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_Left (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 if (Reset_Left_PI)
	 {
	 	Bias = 0.0f;
	 	Pwm = 0.0f;
	 	Last_bias = 0.0f;
	 	Reset_Left_PI = 0;
	 }
	 Bias=Target-Encoder;                					//计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   	//增量式PI控制器
	 if(Pwm>7800)Pwm=7800;
	 if(Pwm<-7800)Pwm=-7800;
	 Last_bias=Bias;	                   					//保存上一次偏差 
	 return Pwm;                         					//增量输出
}


int Incremental_PI_Right (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 if (Reset_Right_PI)
	 {
	 	Bias = 0.0f;
	 	Pwm = 0.0f;
	 	Last_bias = 0.0f;
	 	Reset_Right_PI = 0;
	 }
	 Bias=Target-Encoder;                					//计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   	//增量式PI控制器
	 if(Pwm>7800)Pwm=7800;
	 if(Pwm<-7800)Pwm=-7800;
	 Last_bias=Bias;	                   					//保存上一次偏差 
	 return Pwm;                         					//增量输出
}

void Reset_Velocity_PI(void)
{
	Reset_Left_PI = 1;
	Reset_Right_PI = 1;
	MotorA.Motor_Pwm = 0.0f;
	MotorB.Motor_Pwm = 0.0f;
}
/**************************************************************************
Function: Press the key to modify the car running state
Input   : none
Output  : none
函数功能：按键修改小车运行状态
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{
    u8 tmp;

    tmp = key_scan(200);

    if (Menu_Active)
    {
        Flag_Stop = 1;

        if (tmp == USEKEY_single_click)
        {
            Menu_SelectionIndex =
                (uint8_t)((Menu_SelectionIndex + 1U) %
                          MENU_MODE_COUNT);
            Menu_Selection =
                Menu_Items[Menu_SelectionIndex].mode;
        }
        else if (tmp == USEKEY_double_click)
        {
            Run_Mode = Menu_Selection;
            Menu_Active = 0U;
            Flag_Stop = 1;
        }
        return;
    }

    if (tmp == USEKEY_long_click)
    {
        Flag_Stop = 1;
        Reset_Velocity_PI();
        IR_Differential_OneLap_Reset();
        TurnCalibration_Reset();
        StraightTurnTest_Reset();
        ball_static_task_stop();
        ball_hold_lap_stop();

        if (Run_Mode == RUN_MODE_STRAIGHT_TURN ||
            Run_Mode == RUN_MODE_BALL_LAP ||
            Run_Mode == RUN_MODE_DRIBBLE ||
            Run_Mode == RUN_MODE_BALL_HOLD_LAP ||
            Run_Mode == RUN_MODE_IMU_DEBUG ||
            Run_Mode == RUN_MODE_BALL_STATIC ||
            Run_Mode == RUN_MODE_SERVO_ADJUST)
        {
            menu_select_mode((uint8_t)Run_Mode);
        }
        else
        {
            menu_select_mode(RUN_MODE_MENU_DEFAULT);
        }
        Menu_Active = 1U;
        return;
    }

    if (tmp == USEKEY_double_click &&
        Run_Mode == RUN_MODE_TURN_CAL)
    {
        if (Flag_Stop)
        {
            Reset_Velocity_PI();
            TurnCalibration_Start();
        }
        else
        {
            Reset_Velocity_PI();
            TurnCalibration_Stop();
        }
        return;
    }

    if (tmp == USEKEY_single_click)
    {
        if (Run_Mode == RUN_MODE_TURN_CAL)
        {
            if (Flag_Stop)
            {
                TurnCalibration_IncreaseRadius();
            }
            else
            {
                Reset_Velocity_PI();
                TurnCalibration_Stop();
            }
        }
        else if (Run_Mode == RUN_MODE_STRAIGHT_TURN ||
                 Run_Mode == RUN_MODE_BALL_LAP ||
                 Run_Mode == RUN_MODE_DRIBBLE)
        {
            if (Flag_Stop)
            {
                Reset_Velocity_PI();
                if (Run_Mode == RUN_MODE_BALL_LAP ||
                    Run_Mode == RUN_MODE_DRIBBLE)
                {
                    StraightTurnTest_StartWithPostLap(
                        STRAIGHT_TURN_BALL_SPEED_MPS,
                        STRAIGHT_TURN_BALL_ACCELERATION_MPS2,
                        STRAIGHT_TURN_BALL_POST_LAP_DISTANCE_M);
                }
                else
                {
                    StraightTurnTest_Start(
                        STRAIGHT_TURN_FAST_SPEED_MPS,
                        STRAIGHT_TURN_FAST_ACCELERATION_MPS2);
                }
            }
            else
            {
                Reset_Velocity_PI();
                StraightTurnTest_Stop();
            }
        }
        else if (Run_Mode == RUN_MODE_IMU_DEBUG)
        {
            Flag_Stop = 1;
            imu_request_yaw_zero();
        }
        else if (Run_Mode == RUN_MODE_BALL_STATIC)
        {
            Flag_Stop = 1;
            if (ball_static_task_is_running() != 0U)
            {
                ball_static_task_stop();
            }
            else
            {
                (void)ball_static_task_start();
            }
        }
        else if (Run_Mode == RUN_MODE_BALL_HOLD_LAP)
        {
            Reset_Velocity_PI();
            if (ball_hold_lap_state == BALL_HOLD_LAP_READY ||
                ball_hold_lap_state == BALL_HOLD_LAP_DONE ||
                ball_hold_lap_state == BALL_HOLD_LAP_ABORTED ||
                ball_hold_lap_state == BALL_HOLD_LAP_FAULT)
            {
                ball_hold_lap_start();
            }
            else
            {
                ball_hold_lap_stop();
            }
        }
        else if (Run_Mode == RUN_MODE_SERVO_ADJUST)
        {
            Flag_Stop = 1;
        }
        else
        {
            if (Flag_Stop && Run_Mode == RUN_MODE_ONE_LAP)
            {
                IR_Differential_OneLap_Reset();
            }
            Flag_Stop = !Flag_Stop;
        }
    }
}
/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
函数功能：限幅函数
入口参数：幅值
返回  值：无
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;
}
/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
函数功能：限制PWM值
入口参数：幅值
返回  值：无
**************************************************************************/
void Limit_Pwm(int amplitude)
{
    MotorA.Motor_Pwm=target_limit_float(MotorA.Motor_Pwm,-amplitude,amplitude);
    MotorB.Motor_Pwm=target_limit_float(MotorB.Motor_Pwm,-amplitude,amplitude);
}
