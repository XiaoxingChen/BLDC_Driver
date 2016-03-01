/**
******************************************************************************
* @file    Model_BLDC.h
* @author  Chenxx
* @version V1.0
* @date    2016-02-19
* @brief   This file defines the BLDC physical model.
*						This file is for PC.
******************************************************************************/
#ifndef _CMODEL_BLDC_
#define _CMODEL_BLDC_
#include <stdint.h>
class CBLDC_Driver;
class CModel_BLDC
{
public:
	friend CBLDC_Driver;
	struct MotParamTypDef
	{
		double R_loop;	//include R_coil, R_Mosfet and R_diode
		double L_coil;
		double U_power;
		double K_e;		//back EMF
		double J_r; //Inertial of rotator
		double J_l; //Inertial of load
		double A_f; //Amplitude of friction
	};
	struct MidValTypDef
	{
		double I_coil;
		double U_duty;	//U_power* pwmduty
		double U_m; //U_duty - E_bemf
		double T_m;	//T - Tl
		double T_l;	//Torque from load
		//double T_f;	//friction force
	};
	struct phyValTypDef 
	{
		double Pos;
		double Vel;
		double Acc;
		double stepLen;
		double PwmDuty;	//[-1, 1]
	};
	
	void update();
	phyValTypDef Get_phyVal_() const;
	double Get_coilCur_() const;	//coil current
	void Tm_friction_limit();
	CModel_BLDC();
	~CModel_BLDC();

private:
	void set_PwmDutyF(double);
	MotParamTypDef MotParam_;
	MidValTypDef MidVal_;
	phyValTypDef phyVal_;
};

extern CModel_BLDC Mot_Model;
#endif
//end of file
