/*
 * autonomous.h
 *
 *  Created on: Feb 6, 2018
 *      Author: Scouting1792-PC
 */
#include <WPILib.h>
#include <AHRS.h>
#include <ctre/Phoenix.h>
#include <Joystick.h>
#include <Talon.h>
#include <Timer.h>
#include <PIDController.h>
#include <DigitalInput.h>

#ifndef SRC_AUTONOMOUS_H_
#define SRC_AUTONOMOUS_H_

class AutoManager {
private:
	//LiftManager *liftManager;

	WPI_TalonSRX *srx1;  //has enc
	WPI_TalonSRX *srx2;  //left
	WPI_TalonSRX *srx3;

	WPI_TalonSRX *srx4;  //has enc
	WPI_TalonSRX *srx5;  //right
	WPI_TalonSRX *srx6;

	WPI_TalonSRX *intake1; //intake motor (left?)
	WPI_TalonSRX *intake2; //intake motor (right?)

	WPI_TalonSRX *liftSRX1; //lift motor top
	WPI_TalonSRX *liftSRX2; //lift motor bottom

	AHRS *ahrs;

	double pi;
	double constant;

	Timer *timer;

public:
	//public AutoManager(LiftManager *lift)
	AutoManager();
	void Auto1();
	void Auto2();
	void Auto3();
	void Auto4();
	void Auto5();
};



#endif /* SRC_AUTONOMOUS_H_ */
