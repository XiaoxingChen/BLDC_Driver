/**
******************************************************************************
* @file    BLDC_Driver.h
* @author  Chenxx
* @version V1.0
* @date    2016-02-22
* @brief   This file defines the BLDC physical model.
*			This file is for PC.
******************************************************************************/
#ifndef _BLDC_DRIVER_H_
#define _BLDC_DRIVER_H_
#include <stdint.h>
#include "Cpid.h"
class CModel_BLDC;
class CBLDC_Driver
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
	CBLDC_Driver();
	~CBLDC_Driver();
	void Init();
	void update();
	Cpid* Get_vctrl_();
	Cpid* Get_pctrl_();

	//
	// little function for ucp
	//
	uint16_t Get_Pos_();
	uint16_t Get_Vel_();
	uint16_t Get_Acc_();
	uint16_t Get_Cur_();
	uint16_t Get_PwmDuty10k();
	void Set_PwmDuty10k(int16_t);

	//
	// variables 
	//
	uint8_t ctrl_mode_;			//control mode
	uint16_t tar_ctrl_val_; //target control value

private:
	void SetPwm16bDuty(int32_t duty);
	int16_t GetPwm16bDuty();
	Cpid* vctrl_;
	Cpid* pctrl_;
	CModel_BLDC *MotModel_;

	uint16_t pwmDuty_raw_;
	uint8_t rotateDir_;	// 0: encoder++, 1: encoder--
};

extern CBLDC_Driver driver;
#endif
//end of file
