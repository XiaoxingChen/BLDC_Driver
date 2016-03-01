/**
******************************************************************************
* @file    BLDC_Driver.h
* @author  Chenxx
* @version V1.0
* @date    2016-01-29
* @brief   This file achieve the driver for brushles DC motor class
******************************************************************************/
#ifndef __BLDC_DRIVER_H
#define __BLDC_DRIVER_H
#include "stm32f10x.h"
class pwmGtor;
class BLDC_Hall;
class incEncoder;
class Cpid;

class BLDC_Driver
{
public:
	//
	//control mode enum
	//
	enum ctrl_mode
	{
		open_loop = 0,//open loop control
		pos_ctrl = 1, //position control
		vel_ctrl = 2,	//velocity control
		cur_ctrl = 3,	//current control
		posinc_ctrl = 4
	};
	
	//
	// embeded class nPwmIO
	//
	class nPwmIO
	{
	public:
		nPwmIO(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
		void ClkConfig();
		void PinConfig();
	
		void Write_IO(uint16_t);
	private:
		GPIO_TypeDef* GPIOx_;
		uint16_t PIN_POS;
	};
	
	//
	//InitTypdef
	//
	struct InitTypdef
	{
		TIM_TypeDef * pwm_Timer;
		uint8_t pwm_TimCh;
		uint16_t pwm_Freq;
		GPIO_TypeDef * npwmIO_Port;
		uint16_t npwmIO_Pin;
		GPIO_TypeDef * hallIO_Port;
		uint16_t hallIO_Pin;
		TIM_TypeDef * encoder_Timer;
	};
	
	BLDC_Driver();
	void Init(InitTypdef*);
	
	void update();
	void CommutateHall();
	void SetOpenLoopDuty(int16_t duty_phy);
	const BLDC_Hall* Get_hall_() const;
	const incEncoder* Get_encoder_() const;
	Cpid* Get_vctrl_();
	Cpid* Get_pctrl_();
	void Encoder_Overflow();
	int32_t calc_Target_pos(int16_t pos_inc);
	
	//
	// little function
	//
	uint8_t Get_rotateDir_() const;
	uint16_t Get_pwmDuty_phy() const;
	
	//
	// variables 
	//
	uint8_t ctrl_mode_;			//control mode
	uint16_t tar_ctrl_val_; //target control value
	
private:
	void SetPwm16bDuty(int32_t duty);
	int16_t GetPwm16bDuty();

	pwmGtor* pwmIO_;
	nPwmIO* npwmIO_;
	BLDC_Hall* hall_;
	incEncoder* encoder_;
	Cpid* vctrl_;
	Cpid* pctrl_;
	Cpid* pinc_ctrl_;

	uint16_t pwmDuty_raw_;
	uint8_t rotateDir_;	// 0: encoder++, 1: encoder--


public:
	static const u8 TAB_P[6];
	static const u8 TAB_N[6];

};

extern BLDC_Driver BLDC;
#endif
//end of file
