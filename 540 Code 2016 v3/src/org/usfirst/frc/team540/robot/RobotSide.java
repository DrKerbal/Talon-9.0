package org.usfirst.frc.team540.robot;

import edu.wpi.first.wpilibj.SD540;

/**
 * Allows for controlling the robot by each side rather than independent motors to protect the drivetrain
 * @author Uday
 *
 */
public class RobotSide {
	SD540 motor1;
	SD540 motor2;
	boolean invert;
	
	public RobotSide(SD540 m1, SD540 m2, boolean invert){
		motor1 = m1;
		motor2 = m2;
		this.invert = invert;
	}
	public void setValue(double speed){
		motor1.set((invert?-1:1)*speed);
		motor2.set((invert?-1:1)*speed);
	}
}
