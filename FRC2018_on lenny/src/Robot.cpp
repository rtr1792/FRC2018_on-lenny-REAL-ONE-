#include <iostream>
#include <string>
#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <Joystick.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <climber.h>
#include <drive.h>
#include <intake.h>
#include <lift.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Drive/DifferentialDrive.h>
#include <DriverStation.h>
#include <PowerDistributionPanel.h>
#include <autonomous.h>

class Robot : public frc::IterativeRobot {

public:
	Robot() {
	this->driveManager = new DriveManager();
	this->climberManager = new ClimberManager();
	this->liftManager = new LiftManager();
	this->intakeManager = new IntakeManager();
	this->autoManager = new AutoManager();
	}
private:
	DriveManager *driveManager;
	ClimberManager *climberManager;
	LiftManager *liftManager;
	IntakeManager *intakeManager;
	AutoManager *autoManager;

	frc::Joystick stick { 0 };
//	frc::XboxController xbox { 1 };
	frc::PowerDistributionPanel pdp;

	void RobotInit() {
	}


	void AutonomousInit() override {

	}

	void AutonomousPeriodic() {
		//this->driveManager->Drive(0);
		this->autoManager->Auto1();
	}

	void TeleopInit() {}

	void TeleopPeriodic() {
		bool test;
		test = frc::SmartDashboard::GetNumber("batterySet", 0);
		frc::SmartDashboard::PutNumber("battery#",test);
		frc::SmartDashboard::PutNumber("battery voltage",pdp.GetVoltage());

		//creates a arcade drive on talons 1-4 using joystick axies 1-2
		this->driveManager->driveTrain();

		//has a climber on talon 9 controlled with button 11
		this->climberManager->Climber();

		//has a intake on talons 7,8 controlled with button 3 (in) and button 4 (out)
		this->intakeManager->Intake();

		//has a lift with talon 10 controlled with button 5 (up) and 6 (down)
		this->liftManager->Lift();

/*		bool x1 = xbox.GetRawButton(1);
		double x2 = xbox.GetRawAxis(5);

		frc::SmartDashboard::PutNumber("x1",x1);
		frc::SmartDashboard::PutNumber("x2",x2); */
}

	void TestPeriodic() {}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
};

START_ROBOT_CLASS(Robot)
