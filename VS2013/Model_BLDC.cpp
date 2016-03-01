/**
******************************************************************************
* @file    Model_BLDC.cpp
* @author  Chenxx
* @version V1.0
* @date    2016-02-19
* @brief   This file defines the BLDC physical model.
*						This file is for PC.
******************************************************************************/
#include "Model_BLDC.h"
#include <math.h>
//
//CModel_BLDC::CModel_BLDC()
//
CModel_BLDC::CModel_BLDC()
{
	MotParam_.J_l = 0;
	MotParam_.J_r = 1E-5;	//from data sheet 135gcm^2 
	MotParam_.K_e = 0.03;	//from data sheet 25mNm/A
	MotParam_.L_coil = 1E-3;	//from data sheet
	MotParam_.R_loop = 13;			//calculate from dead band
	MotParam_.U_power = 12;	//12v
	MotParam_.A_f = 0.002;	//calculate from power lost

	phyVal_.Pos = 0;
	phyVal_.Vel = 0;
	phyVal_.Acc = 0;
	phyVal_.PwmDuty = 0;
	phyVal_.stepLen = 0.001;
}

//
//void CModel_BLDC::update()
//
void CModel_BLDC::update()
{
	static double prev_Vel = 10;
	static double prev_Icoil = 0;
	/*phyVal_.Vel = (double)phyVal_.PwmDuty/40.0;*/

	MidVal_.U_duty = MotParam_.U_power * phyVal_.PwmDuty;
	MidVal_.U_m = MidVal_.U_duty - phyVal_.Vel * MotParam_.K_e*abs(phyVal_.PwmDuty);
	MidVal_.I_coil = (MidVal_.U_m*phyVal_.stepLen + MotParam_.L_coil*prev_Icoil)
		/ (MotParam_.L_coil + MotParam_.R_loop* phyVal_.stepLen);

	
	MidVal_.T_m = MidVal_.I_coil*MotParam_.K_e + MidVal_.T_l;	
	Tm_friction_limit();

	double t1;
	t1 = (0 - phyVal_.Vel) / phyVal_.Acc;
	if (t1 > 0 && t1 < phyVal_.stepLen)
	{
		phyVal_.Pos += phyVal_.Vel*t1;
		phyVal_.Vel = 0;

		/* recalculate T_m */
		MidVal_.T_m = MidVal_.I_coil*MotParam_.K_e + MidVal_.T_l;
		Tm_friction_limit();
	}
	else t1 = 0;

	phyVal_.Acc = MidVal_.T_m / (MotParam_.J_l + MotParam_.J_r);
	phyVal_.Vel += (phyVal_.stepLen - t1) * phyVal_.Acc;
	phyVal_.Pos += (phyVal_.stepLen - t1)* phyVal_.Vel;

	phyVal_.Pos -= (phyVal_.Pos > 2 * M_PI * 31) ? (2 * M_PI * 31) : 0; //limit
	phyVal_.Pos += (phyVal_.Pos < 0) ? (2 * M_PI * 31) : 0;
	prev_Vel = phyVal_.Vel;
	prev_Icoil = MidVal_.I_coil;

}

//
//
//
void CModel_BLDC::Tm_friction_limit()
{
	if (phyVal_.Vel < 1E-3 && phyVal_.Vel > -1E-3)	//velocity == 0
	{
		/* dead band */
		if (MidVal_.T_m < MotParam_.A_f && MidVal_.T_m > -MotParam_.A_f) MidVal_.T_m = 0;
	}
	else if (phyVal_.Vel >= 1E-3)		//v >= 0
	{
		MidVal_.T_m -= MotParam_.A_f;
	}
	else if (phyVal_.Vel <= -1E-3)	//v <= 0
	{
		MidVal_.T_m += MotParam_.A_f;
	}
}
//
//CModel_BLDC::~CModel_BLDC()
//
CModel_BLDC::~CModel_BLDC()
{

}

//
//phyValTypDef Get_phyVal_() const
//
CModel_BLDC::phyValTypDef CModel_BLDC::Get_phyVal_() const
{
	return phyVal_;
}

//
//double CModel_BLDC::Get_coilCur_() const
//
double CModel_BLDC::Get_coilCur_() const
{
	return MidVal_.I_coil;
}

//
//void CModel_BLDC::set_PwmDutyF(double dutyF)
//
void CModel_BLDC::set_PwmDutyF(double dutyF)
{
	phyVal_.PwmDuty = dutyF;
}
CModel_BLDC Mot_Model;
//end of file
