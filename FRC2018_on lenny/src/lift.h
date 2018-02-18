/*
 * lift.h
 *
 *  Created on: Jan 27, 2018
 *      Author: RTR
 */

#ifndef SRC_SUBCLASS_LIFT_H_
#define SRC_SUBCLASS_LIFT_H_

class LiftManager {
private:
	WPI_TalonSRX *srx1;
	WPI_TalonSRX *srx2;

	WPI_TalonSRX *DriveSRX1;
	WPI_TalonSRX *DriveSRX2;

	Joystick *stick;
	XboxController *xbox;

	DigitalInput *limit;
	DigitalInput *limit2;

	Timer *timer;

	int *liftValue;
	int *button3;
	int *button4;
	int *limits;
	double *encoder;
	int *dlimit;
	int *init;
	int *limitOveride;
	int *limitOveride2;
public:
	LiftManager();
	void Lift();
	void Liftmove(int pos);
};

#endif /* SRC_SUBCLASS_LIFT_H_ */
