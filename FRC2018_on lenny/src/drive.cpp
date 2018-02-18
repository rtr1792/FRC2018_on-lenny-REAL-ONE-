/*
 * drive.cpp
 *
 *  Created on: Jan 24, 2018
 *      Author: RTR
 */
#include <iostream>
#include <string>
#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <Joystick.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <SpeedController.h>
#include <Drive/DifferentialDrive.h>
#include <DriverStation.h>
#include "drive.h"
#include <iostream>
#include <Encoder.h>
#include "AHRS.h"
#include <SPI.h>
//n the declaration
bool VelocityControl = false;
double left = 0;
double right = 0;
int kTimeoutMs = 10;
bool CurrentControl = false;

DriveManager::DriveManager() {
	srx1 = new WPI_TalonSRX(1);
	srx12 = new WPI_TalonSRX(2);
	srx13 = new WPI_TalonSRX(3);

	srx2 = new WPI_TalonSRX(4);
	srx21 = new WPI_TalonSRX(5);
	srx22 = new WPI_TalonSRX(6);

	if(!VelocityControl){
		m_robotDrive = new DifferentialDrive(*srx1, *srx2);
		m_robotDrive->SetSafetyEnabled(true);
	}
	else{
		double kPIDLoopIdx = 0;
		bool Inverted = true;

		double Fgain = 0.1532;
		double Pgain = 0.0;
		double Igain = 0.0;
		double Dgain = 0.0;

		//SRX1
		srx1->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
		srx1->SetSensorPhase(true);
		srx1->SetInverted(Inverted);
		srx1->ConfigAllowableClosedloopError(0, 0, kTimeoutMs);


		/* set the peak and nominal outputs, 12V means full */
		srx1->ConfigNominalOutputForward(0, kTimeoutMs);
		srx1->ConfigNominalOutputReverse(0, kTimeoutMs);
		srx1->ConfigPeakOutputForward(12, kTimeoutMs);
		srx1->ConfigPeakOutputReverse(-12 , kTimeoutMs);

		/* set closed loop gains in slot0 */
		srx1->Config_kF(kPIDLoopIdx, Fgain, kTimeoutMs);
		srx1->Config_kP(kPIDLoopIdx, Pgain, kTimeoutMs);
		srx1->Config_kI(kPIDLoopIdx, Igain, kTimeoutMs);
		srx1->Config_kD(kPIDLoopIdx, Dgain, kTimeoutMs);

		//SRX2
		srx2->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
		srx2->SetSensorPhase(true);
		srx2->SetInverted(Inverted);
		srx2->ConfigAllowableClosedloopError(0, 0, kTimeoutMs);


		/* set the peak and nominal outputs, 12V means full */
		srx2->ConfigNominalOutputForward(0, kTimeoutMs);
		srx2->ConfigNominalOutputReverse(0, kTimeoutMs);
		srx2->ConfigPeakOutputForward(12, kTimeoutMs);
		srx2->ConfigPeakOutputReverse(-12 , kTimeoutMs);

		/* set closed loop gains in slot0 */
		srx2->Config_kF(kPIDLoopIdx, Fgain, kTimeoutMs);
		srx2->Config_kP(kPIDLoopIdx, Pgain, kTimeoutMs);
		srx2->Config_kI(kPIDLoopIdx, Igain, kTimeoutMs);
		srx2->Config_kD(kPIDLoopIdx, Dgain, kTimeoutMs);
	}


	srx12->Set(ControlMode::Follower, 1);
	srx13->Set(ControlMode::Follower, 1);
	srx21->Set(ControlMode::Follower, 4);
	srx22->Set(ControlMode::Follower, 4);

	//Left Current Enabling
	srx1->EnableCurrentLimit(CurrentControl);
	srx12->EnableCurrentLimit(CurrentControl);
	srx13->EnableCurrentLimit(CurrentControl);

	//Right Current Enabling
	srx2->EnableCurrentLimit(CurrentControl);
	srx21->EnableCurrentLimit(CurrentControl);
	srx22->EnableCurrentLimit(CurrentControl);

	//Left Continuous Current Limits
	srx1->ConfigContinuousCurrentLimit(40, kTimeoutMs);
	srx12->ConfigContinuousCurrentLimit(40, kTimeoutMs);
	srx13->ConfigContinuousCurrentLimit(40, kTimeoutMs);

	//Right Continuous Current Limits
	srx2->ConfigContinuousCurrentLimit(40, kTimeoutMs);
	srx21->ConfigContinuousCurrentLimit(40, kTimeoutMs);
	srx22->ConfigContinuousCurrentLimit(40, kTimeoutMs);

	this->stick = new Joystick{ 0 };
	xbox = new XboxController { 1 };

	rightStickValue = new double;
	leftStickValue = new double;
	vel1 = new double;
	vel2 = new double;
	dis = new double;
	dis2 = new double;
	init = new int;
	one = new int;

	ahrs = new AHRS(SPI::Port::kMXP);
	ahrs->Reset();

}


double want = 0;
double gyro = 0;
double turnk = -0.3;
double tiltk = -0.01;
double x = 0;
double z = 0;

double m1;
double m2;
double m3;
double m4;
double m5;
double m6;

double c1;
double c2;
double c3;
double c4;
double c5;
double c6;

double encRot;
double encRot2;

double distance;
double distance2;

float DispX;
float DispY;
float DispZ;

void DriveManager::driveTrain() {

	if (stick->GetRawAxis(1) < 0.05 and stick->GetRawAxis(1) > -0.05) {
		x = 0;
	}
	else {
		x = stick->GetRawAxis(1);
	}

	if (stick->GetRawAxis(2) < 0.05 and stick->GetRawAxis(2) > -0.05) {
		z = 0;
	}
	else {
		z = stick->GetRawAxis(2)*0.75;
	}


			if (stick->GetRawButton(1) and !stick->GetRawButton(2)) {
				x = x * 0.5;
				z = z * 0.5;
			}
			else if (stick->GetRawButton(1) and stick->GetRawButton(2)) {
				x = x * 0.5;
				z = 0;
			}

			if (stick->GetRawButton(2)) {
				z = 0;
			}


	*vel1 = srx1->GetSensorCollection().GetQuadratureVelocity();
	*vel2 = -srx2->GetSensorCollection().GetQuadratureVelocity();
	*dis = srx1->GetSensorCollection().GetQuadraturePosition();
	*dis2 = -srx2->GetSensorCollection().GetQuadraturePosition();

	frc::SmartDashboard::PutNumber("velocity1",*vel1);
	frc::SmartDashboard::PutNumber("velocity2",*vel2);
	frc::SmartDashboard::PutNumber("distance",*dis);
	frc::SmartDashboard::PutNumber("distance2",*dis2);

	if (stick->GetRawButton(5)) {
		srx1->GetSensorCollection().SetQuadraturePosition(0,4);
		srx2->GetSensorCollection().SetQuadraturePosition(0,4);
	}
	//4000 = one rotation ?4096
	//diameter = 3.94 in
	//circumfrece = 24.7432

	encRot = (1.0 * srx1->GetSensorCollection().GetQuadraturePosition() / 4096);
	frc::SmartDashboard::PutNumber("encRotations",encRot);
	distance = (encRot * 12.566);
	frc::SmartDashboard::PutNumber("distanceInches",distance);

	encRot2 = (1.0 * -srx2->GetSensorCollection().GetQuadraturePosition() / 4096);
	frc::SmartDashboard::PutNumber("encRotations2",encRot2);
	distance2 = (encRot2 * 12.566);
	frc::SmartDashboard::PutNumber("distanceInches2",distance2);

	m1 = srx1->Get();
	m2 = srx12->Get();
	m3 = srx13->Get();
	m4 = srx2->Get();
	m5 = srx21->Get();
	m6 = srx22->Get();

	c1 = srx1->GetOutputCurrent();
	c2 = srx12->GetOutputCurrent();
	c3 = srx13->GetOutputCurrent();
	c4 = srx2->GetOutputCurrent();
	c5 = srx21->GetOutputCurrent();
	c6 = srx22->GetOutputCurrent();

	frc::SmartDashboard::PutNumber("m1",m1);
	frc::SmartDashboard::PutNumber("m2",m2);
	frc::SmartDashboard::PutNumber("m3",m3);
	frc::SmartDashboard::PutNumber("m4",m4);
	frc::SmartDashboard::PutNumber("m5",m5);
	frc::SmartDashboard::PutNumber("m6",m6);

	frc::SmartDashboard::PutNumber("c1",c1);
	frc::SmartDashboard::PutNumber("c2",c2);
	frc::SmartDashboard::PutNumber("c3",c3);
	frc::SmartDashboard::PutNumber("c4",c4);
	frc::SmartDashboard::PutNumber("c5",c5);
	frc::SmartDashboard::PutNumber("c6",c6);

	//clockwise is positive
	double Yaw = ahrs->GetYaw();
	double Pitch = ahrs->GetPitch();
	double Roll = ahrs->GetRoll();
	gyro = ahrs->GetAngle();
	frc::SmartDashboard::PutNumber("Yaw", Yaw);
	//frc::SmartDashboard::PutString("Firmware", ahrs->GetFirmwareVersion());
	frc::SmartDashboard::PutNumber("Pitch", Pitch);
	frc::SmartDashboard::PutNumber("Roll", Roll);
	frc::SmartDashboard::PutNumber("Angle", gyro);

	if (stick->GetRawButton(6)) {
		ahrs->Reset();
		want = gyro;
	}

	if (stick->GetRawButton(2)) {
		if (z == 0) {
			z = ((gyro - want) * turnk);
		}
		else {
			want = gyro;
		}
	}
	frc::SmartDashboard::PutNumber("want",want);

	if(Pitch > 45 || -45 > Pitch){
		x = x + (gyro) * tiltk;
	}

	if(VelocityControl){
		left = x + z;
		right = x - z;
		left = ((left*(5330/5.45)*4096)/600); //Percentage * 5330RPM of CIM / GearRatio * 4096units in 1 rotation / 600 to units per ms
		right = ((right*(5330/5.45)*4096)/600); //Percentage * 5330RPM of CIM / GearRatio * 4096units in 1 rotation / 600 to units per ms
		srx1->Set(ControlMode::Velocity, left);
		srx2->Set(ControlMode::Velocity, right);
		frc::SmartDashboard::PutNumber("Left Side Error", srx1->GetClosedLoopError(0));
		frc::SmartDashboard::PutNumber("Right Side Error", srx2->GetClosedLoopError(0));
	}
	else{
		m_robotDrive->ArcadeDrive(-x, z);
	}

	DispX = ahrs->GetDisplacementX();
	frc::SmartDashboard::PutNumber("Displacement X", DispX);
	DispY = ahrs->GetDisplacementY();
	frc::SmartDashboard::PutNumber("Displacement Y", DispY);
	DispZ = ahrs->GetDisplacementZ();
	frc::SmartDashboard::PutNumber("Displacement Z", DispZ);

}
void DriveManager::Drive(int speed) {
	m_robotDrive->ArcadeDrive(0, 0);

}


