
package org.usfirst.frc.team540.robot;

import java.io.BufferedReader;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.SD540;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	Joystick leftJoystick, rightJoystick, PSP;
	SD540 frontLeft, frontRight, rearLeft, rearRight, shooter, ballMagR, ballMagL, ruweRail, magLight;
	RobotSide rightSide, leftSide;
	Solenoid first, second, firstB, secondB;
	PowerDistributionPanel pdp;
	AnalogGyro gyro;
	AnalogInput IR2;
	AnalogInput pressure; //p = (250*(Vout/5V)) - 25
	AnalogInput IR;
	Compressor compressor;
	SendableChooser chooser;
	NetworkTable table;
	Encoder encLeft, encRight, encRuwe;
	//ADXL345_I2C accel;
	//ADXL345_I2C.AllAxes accelerations;
	//double accelerationX;
	//double accelerationY;
	//double accelerationZ;
	double yValLeft, yValRight, PSPStick, rangePrior, IRdist2, angle, PSI, IRdist;
	boolean pressureSwitch, crossing, readyToTrack, ruweUp, autoShift, speedMode, facingForwards, leftTrigger, debounce, rightTrigger, xboxX, xboxA,xboxB, useIR, shooterOn, teleTrack;
	final String defaultAuto = "Default";
	final String mode0 = "Mode 0";		
	final String mode1 = "Mode 1";
	final String mode2 = "Mode 2";
	final String mode3 = "Mode 3";
	final String mode4 = "Mode 4";
	final String mode5 = "Mode 5";
	final String mode6 = "Mode 6";
	String autoSelected;
	static double angle2 = -120;
	static double rotate1 = 0.15;
	static double P = 1.5;
	static double Pexp = -1.0;
	static double I = 0.0;
	static double Iexp = 0.0;
	static double D = 2.0;
	static double Dexp = -3.0;
	static int currSetPoint = 0;
	static double lastError; 				//Records last error for derivative
	static double errorSum = 0; 			//Sum of the errors for integral
	static double errorChange; 				//Derivative of error
	static double errorTolerance = 20;		//Used to create a deadzone around the setPoint
	static double error;
	static int counter = -1;
	static double forward1 = 0.1;
	static double d = 5;
	static boolean toggle = true;
	static double lastCameraVal = 0;
	double forwardVal = .5;
	double teleAngle = -113;
	static long startTime;
	double[] vals = {0,0};
	public static double[] tableVals = {0,0};

	public void robotInit() {
		(new Thread(new UDP())).start();
		leftJoystick = new Joystick(0);
		rightJoystick = new Joystick(1);
		PSP = new Joystick(2);
		//Joysticks

		frontLeft = new SD540(3);//1
		frontRight = new SD540(0);//3
		rearLeft = new SD540(2);//0
		rearRight = new SD540(1);//2
		shooter = new SD540(4);
		ballMagR = new SD540(6);
		ballMagL = new SD540(5);
		ruweRail = new SD540(7);
		//SD540s

		magLight = new SD540(9);
		//Light controller

		rightSide = new RobotSide(frontRight, rearRight, true);
		leftSide = new RobotSide(frontLeft, rearLeft, false);

		pressureSwitch = false;
		crossing = false;
		readyToTrack = false;
		ruweUp = false;
		autoShift = false;//set to false cause no encoders
		facingForwards = false;
		debounce = false;
		leftTrigger = false;
		rightTrigger = false;
		teleTrack = false;
		xboxX = false;
		xboxA = false;
		xboxB = false;
		useIR = false;
		shooterOn = false;
		//booleans

		counter = -1; //Counter for auto

		first = new Solenoid(1);
		second = new Solenoid(2);
		firstB = new Solenoid(0);
		secondB = new Solenoid(3);
		//solenoids

		speedMode = first.get();
		//boolean - Set speed mode to the current solenoid setting

		IR2 = new AnalogInput(1);
		gyro = new AnalogGyro(0);
		gyro.reset();
		gyro.calibrate();
		pressure = new AnalogInput(2);
		IR = new AnalogInput(3);
		//Analog Sensors

		table = NetworkTable.getTable("cameratrack");
		//Initialize the Network Table from raspberry Pi

		compressor = new Compressor();
		pdp = new PowerDistributionPanel();
		//Other Stuff

		SmartDashboard.putNumber("Rotate 1", rotate1);
		SmartDashboard.putNumber("P", P);
		SmartDashboard.putNumber("Pexp", Pexp);
		SmartDashboard.putNumber("I", I);
		SmartDashboard.putNumber("Iexp", Iexp);
		SmartDashboard.putNumber("D", D);
		SmartDashboard.putNumber("Dexp", Dexp);
		SmartDashboard.putNumber("AUTO Counter", counter);
		SmartDashboard.putNumber("forwardVal", forwardVal);
		SmartDashboard.putNumber("teleAngle", teleAngle);
		//Adds the above values to the Dashboard
		
		/*
		encLeft = new Encoder(1, 1, false);
		encRight= new Encoder(2, 2, true);
		encRuwe = new Encoder(3, 3, false);
		Initialize and Reset the Encoders
		encLeft.reset();
		encRight.reset();
		encRuwe.reset();
		encLeft.setSamplesToAverage(2);
		encRight.setSamplesToAverage(2);
		encRuwe.setSamplesToAverage(2);
		//Encoders
		 */


		chooser = new SendableChooser();
		chooser.addDefault("Do Nothing (Default)", defaultAuto);
		chooser.addObject("Low Bar (CAN USE)", mode1);
		chooser.addObject("Moat/Rough Terrain (CAN USE)", mode2);
		chooser.addObject("Moat/Rough Terrain (More Oomph) (CAN USE)", mode3);
		chooser.addObject("Mobility Camera Track (CAN USE TESTING)", mode4);
		chooser.addObject("Low Bar Turn and Camera Track (CAN USE TESTING)", mode5);
		chooser.addObject("Low Bar Turn and Gyro Rotate (CAN USE TESTING)", mode6);

		SmartDashboard.putData("Auto choices", chooser);
		//Auto Modes
		
	}

	public double[] PID(double centerX)
	{
		//P I and D have to have two dashboard values for scientific notation to get more accurate values
		double[] out = new double[2];
		error = centerX+70; //Calculate Error
		double integral = I * Math.pow(10, Iexp);
		double prop = P * Math.pow(10, Pexp);
		double deriv = P * Math.pow(10, Pexp);
		if(Math.abs(error) < errorTolerance) //Creates deadzone for error
		{

			error = 0;
			errorSum = 0;
		}

		errorSum = errorSum + error; //Calculate integral

		errorChange = error - lastError; //Calculates derivative

		lastError = error; //Rewrites last error for calculating derivative at next refresh

		if(facingForwards){
			out[0] = (prop*error) + (integral*errorSum) + (deriv*errorChange); //Calculates Motor Speed based on proportional, integral, and derivative control
			out[1] = -1*((prop*error) + (integral*errorSum) + (deriv*errorChange));
		}else{
			out[1] = (prop*error) + (integral*errorSum) + (deriv*errorChange); //Calculates Motor Speed based on proportional, integral, and derivative control
			out[0] = -1*((prop*error) + (integral*errorSum) + (deriv*errorChange));
		}
		if(Math.abs(out[1])>0.75){
			out[1] = out[1]/Math.abs(out[1]) *.75;
		}
		if(Math.abs(out[0])>0.75){
			out[0] = out[0]/Math.abs(out[0]) *.75;
		}
		return out;
	}
	public double[] calcCamera(double centerX){
		double[] powers = new double[2];
		if(Math.abs(centerX) < 20)
		{
			powers[0] = 0;
			powers[1] = 0;
		}
		else
		{
			double value = rotate1*Math.log(Math.abs(centerX));
			if(facingForwards){

				powers[0] = 0 +((centerX)/Math.abs(centerX))*value;
				powers[1] = 0 -((centerX)/Math.abs(centerX))*value;
			}else{
				powers[1] = 0 +((centerX)/Math.abs(centerX))*value;
				powers[0] = 0 -((centerX)/Math.abs(centerX))*value;
			}
			if(Math.abs(powers[1])>0.5){
				powers[1] = powers[1]/Math.abs(powers[1]) *.5;
			}
			if(Math.abs(powers[0])>0.5){
				powers[0] = powers[0]/Math.abs(powers[0]) *.5;
			}
		}	

		return powers;
	}

	public double[] calcPath(double gyro, double angle2)
	{
		this.angle2 = angle2;
		double[] powers = new double[2];
		if(Math.abs(gyro-angle2) < 10)
		{
			powers[0] = 0;
			powers[1] = 0;
		}
		else
		{
			double value = rotate1*Math.log(Math.abs(gyro - angle2));
			if(facingForwards){
				powers[0] = 0 +((gyro-angle2)/Math.abs(gyro-angle2))*value;
				powers[1] = 0 -((gyro-angle2)/Math.abs(gyro-angle2))*value;
			}else{
				powers[1] = 0 +((gyro-angle2)/Math.abs(gyro-angle2))*value;
				powers[0] = 0 -((gyro-angle2)/Math.abs(gyro-angle2))*value;
			}
		}	

		return powers;
	}
	public static double[] calcDistance(double avgEnc)
	{
		double[] powers = new double[2];
		if(Math.abs(avgEnc-d) < 1){
			powers[0] = 0;
			powers[1] = 0;
		}else{
			double value = forward1*Math.log(Math.abs(avgEnc - d));
		}

		return powers;
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the getString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the switch structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
	public void autonomousInit() {
		autoSelected = (String) chooser.getSelected();
		//		autoSelected = SmartDashboard.getString("Auto Selector", defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {
		autoSelected = (String) chooser.getSelected();
		angle = ((gyro.getAngle()%360)>180)?(gyro.getAngle()%360)-360:(gyro.getAngle()%360);
		SmartDashboard.putNumber("AUTO Counter", counter);
		tableVals[1] = table.getNumber("centerX", 0);
		table.putNumber("angle", gyro.getAngle());
		table.putNumber("PSI", PSI);
		table.putBoolean("ball", IRdist<20);
		table.putBoolean("aligned", tableVals[1]<20);
		table.putBoolean("compressor", compressor.getClosedLoopControl());
		switch(autoSelected) {

		case mode1:
			//Drives the robot forward over the lowbar, and then crosses and stops
			if(toggle){
				leftSide.setValue(.6);
				rightSide.setValue(.6);
			}else{
				leftSide.setValue(0);
				rightSide.setValue(0);
			}
			Timer.delay(3.5);
			toggle = false;
			break;

		case mode2:
			//Moves across the moat
			if(toggle){
				leftSide.setValue(-1);
				rightSide.setValue(-1);
			}else{
				leftSide.setValue(0);
				rightSide.setValue(0);
			}
			Timer.delay(2.75);
			toggle = false;
			break;
		case mode3:
			//Moves across the moat for a longer time 
			if(toggle){
				leftSide.setValue(-1);
				rightSide.setValue(-1);
			}else{
				leftSide.setValue(0);
				rightSide.setValue(0);
			}
			Timer.delay(3.5);
			toggle = false;
			break;	

		case mode4:
			//Moves across the moat and tracks DOES NOT SHOOT
			if(toggle){
				leftSide.setValue(-1);
				rightSide.setValue(-1);
				Timer.delay(2.75);
				lastCameraVal = tableVals[1];
				toggle = false;
				startTime = System.currentTimeMillis();
				counter = 0;

			}else if(counter == 0){
				leftSide.setValue(-.2);
				rightSide.setValue(-.2);
				if(lastCameraVal != (tableVals[1]=table.getNumber("centerX", 0)) || System.currentTimeMillis() - startTime > 4000){

					if(System.currentTimeMillis() - startTime > 2000){
						leftSide.setValue(0);
						rightSide.setValue(0);
						counter = -1;
					}else{
						counter++;
					}
					leftSide.setValue(0);
					rightSide.setValue(0);
				}
			}else if(counter >= 1 && counter < 3){
				leftSide.setValue(0);
				rightSide.setValue(0);
				Timer.delay(0.1);
				vals = PID(tableVals[1]);
				//				angle2 = 0;
				//				angle = ((gyro.getAngle()%360)>180)?(gyro.getAngle()%360)-360:(gyro.getAngle()%360);
				//				vals = calcPath(angle);
				if(vals[0] != 0 && vals[1] != 0)
				{
					leftSide.setValue(vals[0]);
					rightSide.setValue(vals[1]);
					counter = 1;
					Timer.delay(.02);
				}
				else
				{
					counter++;
					startTime = System.currentTimeMillis();
					lastCameraVal = table.getNumber("centerX",0);
					leftSide.setValue(-0.1);
					rightSide.setValue(-0.1);
					Timer.delay(0.1);
				}
			}else if(counter == 3){
				leftSide.setValue(-.3);
				rightSide.setValue(-.2);
				if(lastCameraVal == (tableVals[1]=table.getNumber("centerX", 0)) || System.currentTimeMillis() - startTime > 4000){

					if(System.currentTimeMillis() - startTime > 8000){
						leftSide.setValue(0);
						rightSide.setValue(0);
						counter = -1;
					}else{
						counter++;
					}
					leftSide.setValue(0);
					rightSide.setValue(0);
				}else{
					lastCameraVal = tableVals[1];
					Timer.delay(0.2);
				}
			}else if(counter == 4){
				/*shooter.set(1);
				Timer.delay(2);
				ballMagR.set(1);
				ballMagL.set(-1);
				Timer.delay(1);
				shooter.set(0);
				ballMagR.set(0);
				ballMagL.set(0);
				 */
			}

			//Set Left Side to go forward close to goal
			//Set Right side to go forward close to goal



			//Shoot ball
			break;
		case mode5:
			//Moves across the moat and camera tracks and SHOOTS
			if(toggle){
				leftSide.setValue(0.6);
				rightSide.setValue(0.6);
				Timer.delay(4.7);
				toggle = false;
				counter = 0;

			}
			else if(counter >= 0 && counter < 2) 
			{
				leftSide.setValue(0);
				rightSide.setValue(0);
				vals = calcPath(angle, -110.0);
				//				angle2 = 0;
				//				angle = ((gyro.getAngle()%360)>180)?(gyro.getAngle()%360)-360:(gyro.getAngle()%360);
				//				vals = calcPath(angle);
				if(vals[0] != 0 && vals[1] != 0)
				{
					leftSide.setValue(vals[0]);
					rightSide.setValue(vals[1]);
					counter = 0;
				}
				else if(counter < 1)
				{
					counter++;
					leftSide.setValue(0);
					rightSide.setValue(0);
				}
				else
				{
					counter++;
					startTime = System.currentTimeMillis();
					lastCameraVal = tableVals[1];
					leftSide.setValue(-0.1);
					rightSide.setValue(-0.1);
					Timer.delay(0.1);
				}
			}
			else if(counter == 2){
				
					leftSide.setValue(-.6);
					rightSide.setValue(-.6);
					Timer.delay(2.2);
					counter++;
				
			}			
			else if(counter >= 2 && counter < 4){
				leftSide.setValue(0);
				rightSide.setValue(0);
				Timer.delay(0.1);
				vals = PID(tableVals[1]);
				//				angle2 = 0;
				//				angle = ((gyro.getAngle()%360)>180)?(gyro.getAngle()%360)-360:(gyro.getAngle()%360);
				//				vals = calcPath(angle);
				if(vals[0] != 0 && vals[1] != 0)
				{
					leftSide.setValue(vals[0]);
					rightSide.setValue(vals[1]);
					counter = 2;
					Timer.delay(.02);
				}
				else if(counter < 4)
				{
					counter++;
					leftSide.setValue(vals[0]);
					rightSide.setValue(vals[1]);
					Timer.delay(0.02);
				}
				else
				{
					counter++;
					startTime = System.currentTimeMillis();
					lastCameraVal = tableVals[1];
					leftSide.setValue(-0.1);
					rightSide.setValue(-0.1);
					Timer.delay(0.1);
				}
			}else if(counter == 4){
				shooter.set(1);
				Timer.delay(2);
				ballMagR.set(1);
				ballMagL.set(-1);
				Timer.delay(1);
				shooter.set(0);
				ballMagR.set(0);
				ballMagL.set(0);
				 
			}
			break;
		case mode6:
			//Moves across the moat and Uday sucks
			if(toggle){
				leftSide.setValue(0.6);
				rightSide.setValue(0.6);
				Timer.delay(4.9);
				toggle = false;
				counter = 0;

			}
			else if(counter >= 0 && counter < 2) 
			{
				leftSide.setValue(0);
				rightSide.setValue(0);
				vals = calcPath(angle,-113.0);
				//				angle2 = 0;
				//				angle = ((gyro.getAngle()%360)>180)?(gyro.getAngle()%360)-360:(gyro.getAngle()%360);
				//				vals = calcPath(angle);
				if(vals[0] != 0 && vals[1] != 0)
				{
					leftSide.setValue(vals[0]);
					rightSide.setValue(vals[1]);
					counter = 0;
				}
				else if(counter < 1)
				{
					counter++;
					leftSide.setValue(0);
					rightSide.setValue(0);
				}
				else
				{
					counter++;
					startTime = System.currentTimeMillis();
					lastCameraVal = tableVals[1];
					leftSide.setValue(-0.1);
					rightSide.setValue(-0.1);
					Timer.delay(0.1);
				}
			}
			else if(counter == 2)
			{
				leftSide.setValue(-.6);
				rightSide.setValue(-.6);
				Timer.delay(.5);
				counter++;
			}
			else{
				leftSide.setValue(0);
				rightSide.setValue(0);
				shooter.set(1);
				Timer.delay(2);
				ballMagR.set(1);
				ballMagL.set(-1);
				Timer.delay(1);
				shooter.set(0);
				ballMagR.set(0);
				ballMagL.set(0);
			}

			break;
		default:
			//Does nothing
			leftSide.setValue(0);
			rightSide.setValue(0);
			break;
		}
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		IRdist2 = (-5.6608*Math.pow(IR2.getVoltage(),3)) + (33.18*Math.pow(IR2.getVoltage(), 2)) - (66.156*IR2.getVoltage())+51.569;
		angle = ((gyro.getAngle()%360)>180)?(gyro.getAngle()%360)-360:(gyro.getAngle()%360);
		PSI = 250*(pressure.getAverageVoltage()/5) - 25;
		IRdist = (-5.6608*Math.pow(IR.getVoltage(),3)) + (33.18*Math.pow(IR.getVoltage(), 2)) - (66.156*IR.getVoltage())+51.569;
		yValLeft = leftJoystick.getY();
		yValRight = rightJoystick.getY();
		table.putNumber("angle", gyro.getAngle());
		
		table.putNumber("PSI", PSI);
		table.putBoolean("ball", IRdist<20);
		table.putBoolean("aligned", tableVals[1]<20);
		table.putBoolean("compressor", compressor.getClosedLoopControl());


		Timer.delay(0.01);

		if(rightJoystick.getRawButton(8)){
			angle = 0;
			gyro.reset();
		}
		
		if(rightJoystick.getRawButton(11)){
			toggle = true;
			counter = 0;
		}
		
		if(rightJoystick.getRawButton(10)){
			if(toggle){
				leftSide.setValue(0.6);
				rightSide.setValue(0.6);
				Timer.delay(4.8);
				toggle = false;
				counter = 0;

			}
			else if(counter >= 0 && counter < 2) 
			{
				leftSide.setValue(0);
				rightSide.setValue(0);
				vals = calcPath(angle, teleAngle);
				//				angle2 = 0;
				//				angle = ((gyro.getAngle()%360)>180)?(gyro.getAngle()%360)-360:(gyro.getAngle()%360);
				//				vals = calcPath(angle);
				if(vals[0] != 0 && vals[1] != 0)
				{
					leftSide.setValue(vals[0]);
					rightSide.setValue(vals[1]);
					counter = 0;
				}
				else if(counter < 1)
				{
					counter++;
					SmartDashboard.putNumber("Tele Counter", counter);
					leftSide.setValue(0);
					rightSide.setValue(0);
				}
				else
				{
					counter++;
					startTime = System.currentTimeMillis();
					leftSide.setValue(-0.1);
					rightSide.setValue(-0.1);
					Timer.delay(0.1);
				}
			}
			else if(counter == 2)
			{
				leftSide.setValue(-0.6);
				rightSide.setValue(-0.6);
				Timer.delay(forwardVal);
				counter++;
			}
			else{
				leftSide.setValue(0);
				rightSide.setValue(0);
				shooter.set(1);
				Timer.delay(2);
				ballMagR.set(1);
				ballMagL.set(-1);
				Timer.delay(1);
				shooter.set(0);
				ballMagR.set(0);
				ballMagL.set(0);
			}

		}else if(leftJoystick.getRawButton(10)){
			double[] test = calcPath(angle, 0.0);

			leftSide.setValue(test[0]);
			rightSide.setValue(test[1]);
		}else if(leftJoystick.getRawButton(11)){
			leftSide.setValue(0);
			rightSide.setValue(0);
			//Timer.delay(.08);
			double[] test = PID(tableVals[1]);
			SmartDashboard.putNumber("Left Motor", test[0]);
			SmartDashboard.putNumber("Right Motor", test[1]);
			leftSide.setValue(test[0]);
			rightSide.setValue(test[1]);
			//Timer.delay(.05);

		}else{
			if (Math.abs(yValLeft) < 0.05) {
				if(facingForwards){				
					leftSide.setValue(0);
				}else{
					rightSide.setValue(0);
					//ISSSUES HERE DEBUG ASAP
				}
			}
			else {
				if(facingForwards){				
					leftSide.setValue(yValLeft);
				}else{
					rightSide.setValue(yValLeft);
					//ISSSUES HERE DEBUG ASAP
				}
			}
			//Deadzones the left joystick and left motors


			if (Math.abs(yValRight) < 0.05) {
				if(facingForwards){				
					rightSide.setValue(0);
				}else{
					leftSide.setValue(0);
					//ISSSUES HERE DEBUG ASAP
				}
			}
			else{
				if(facingForwards){
					rightSide.setValue(yValRight);
				}else{
					leftSide.setValue(yValRight);
				}
			}			

		}

		debounce = leftTrigger;
		leftTrigger = leftJoystick.getRawButton(1);
		if (leftTrigger && !debounce) {
			first.set(!first.get());
			second.set(!first.get());
			firstB.set(first.get());
			secondB.set(first.get());
			autoShift = false;
			speedMode = first.get();

			//Manual shifting toggle
		}

		debounce = xboxX;
		xboxX = PSP.getRawButton(3);
		if(xboxX&&!debounce){
			compressor.setClosedLoopControl(!compressor.getClosedLoopControl());

			//shooterOn = !shooterOn;
			//shooter.set(shooterOn?.1:0);
		}

		if (PSP.getRawAxis(3)>0.1&&!shooterOn) {
			shooter.set(1);
		}
		else if(!shooterOn){
			shooter.set(0);
		}
		if (PSP.getRawAxis(2) > 0.1) {

			magLight.set(1);
		}
		else if(!shooterOn){
			magLight.set(0);
		}

		/*		if(leftJoystick.getRawButton(2)){
				autoShift = true;
				Timer.delay(0.05);
			}
			//Switch modes between manual and autoshift
		 */
		/*
		if(rightJoystick.getRawButton(9)){
			teleTrack = false;
			counter = 0;
			toggle = true;
		}
		debounce = teleTrack;
		teleTrack = rightJoystick.getRawButton(8);
		if(teleTrack&&!debounce){
			double[] vals;
			if(toggle){
				leftSide.setValue(-1);
				rightSide.setValue(-1);
				Timer.delay(1);
				lastCameraVal = tableVals[1];
				toggle = false;
				startTime = System.currentTimeMillis();
				counter = 0;

			}else if(counter == 0){
				leftSide.setValue(-.2);
				rightSide.setValue(-.2);
				if(lastCameraVal != (tableVals[1]=table.getNumber("centerX", 0)) || System.currentTimeMillis() - startTime > 4000){

					if(System.currentTimeMillis() - startTime > 2000){
						leftSide.setValue(0);
						rightSide.setValue(0);
						counter = -1;
					}else{
						counter++;
					}
					leftSide.setValue(0);
					rightSide.setValue(0);
				}
			}else if(counter >= 1 && counter < 3){
				leftSide.setValue(0);
				rightSide.setValue(0);
				Timer.delay(0.1);
				vals = PID(tableVals[1]);
//				angle2 = 0;
//				angle = ((gyro.getAngle()%360)>180)?(gyro.getAngle()%360)-360:(gyro.getAngle()%360);
//				vals = calcPath(angle);
				if(vals[0] != 0 && vals[1] != 0)
				{
					leftSide.setValue(vals[0]);
					rightSide.setValue(vals[1]);
					counter = 1;
					Timer.delay(.02);
				}
				else
				{
					counter++;
					startTime = System.currentTimeMillis();
					lastCameraVal = table.getNumber("centerX",0);
					leftSide.setValue(-0.1);
					rightSide.setValue(-0.1);
					Timer.delay(0.1);
				}
			}else if(counter == 3){
				leftSide.setValue(-.3);
				rightSide.setValue(-.2);
				if(lastCameraVal == (tableVals[1]=table.getNumber("centerX", 0)) || System.currentTimeMillis() - startTime > 4000){

					if(System.currentTimeMillis() - startTime > 8000){
						leftSide.setValue(0);
						rightSide.setValue(0);
						counter = -1;
					}else{
						counter++;
					}
					leftSide.setValue(0);
					rightSide.setValue(0);
				}else{
					lastCameraVal = tableVals[1];
					Timer.delay(0.2);
				}
			}else if(counter == 4){
				shooter.set(1);
				Timer.delay(2);
				ballMagR.set(1);
				ballMagL.set(-1);
				Timer.delay(1);
				shooter.set(0);
				ballMagR.set(0);
				ballMagL.set(0);

			}
		}*/
		//negates one value to account for flipped motors

		debounce = rightTrigger;
		rightTrigger = rightJoystick.getRawButton(1);
		if(rightTrigger&&!debounce){
			facingForwards = !facingForwards;
			rightSide.invert = !rightSide.invert;
			leftSide.invert = !leftSide.invert;
		}
		//flips robot orientation

		//Activates shooter motor when button on PSP is pressed

		if (Math.abs(PSP.getRawAxis(5)) < 0.1) {
			ruweRail.set(0);
		}
		else{
			ruweRail.set(PSP.getRawAxis(5));
		}


		PSPStick = (double) PSP.getRawAxis(1);
		if (Math.abs(PSPStick) < 0.1&&!useIR) {
			ballMagR.set(0);
			ballMagL.set(0);
		}
		else if(!useIR){
			ballMagR.set(-PSPStick);
			ballMagL.set(PSPStick);
		}
		debounce = xboxA;
		xboxA = PSP.getRawButton(1);
		if(xboxA&&!debounce){
			useIR = true;
		}else if(IRdist2>15&&IRdist>15&&useIR){
			shooter.set(.4);
		}else if(IRdist2<15&&IRdist>15&&useIR){
			startTime = System.currentTimeMillis();
			ballMagR.set(.4);
			ballMagL.set(-.4);
			shooter.set(.5);
		}else if(IRdist<15&&useIR){
			if(System.currentTimeMillis()-startTime>650&&System.currentTimeMillis()-startTime<900){
				shooter.set(0);
				ballMagR.set(0);
				ballMagL.set(0);
			}else if(System.currentTimeMillis()-startTime>900&&System.currentTimeMillis()-startTime<1150){
				shooter.set(-.5);
			}else if(System.currentTimeMillis()-startTime>1150&&System.currentTimeMillis()-startTime<1300){//1150 and 1300
				ballMagR.set(-.1);
				ballMagL.set(.1);
			}else if(System.currentTimeMillis()-startTime>1300&&System.currentTimeMillis()-startTime<1550){//1300 and 1500
				shooter.set(0);
			}else if(System.currentTimeMillis()-startTime>1550){//1500
				ballMagR.set(0);
				ballMagL.set(0);
				useIR = false;
			}
			/*
			Timer.delay(.65);
			shooter.set(0);
			ballMagR.set(0);
			ballMagL.set(0);
			Timer.delay(.25);
			shooter.set(-.5);
			Timer.delay(0.25);
			ballMagR.set(-.15);
			ballMagL.set(.15);
			Timer.delay(0.15);
			shooter.set(0);
			Timer.delay(0.15);
			ballMagR.set(0);
			ballMagL.set(0);
			useIR = false;
			 */
		}

		debounce = xboxB;
		xboxB = PSP.getRawButton(2);
		if(xboxB&&!debounce){
			useIR = false;
		}

		//Activates Ball Magnet Motor using the Y value of the PSP joystick
		/*
			        	pressureSwitch = compressor.getPressureSwitchValue();
			        	if(pressureSwitch == true) {
			        		relay.set(Relay.Value.kOff);
			        	}			//        	if(pressureSwitch == false) {
			        		relay.set(Relay.Value.kOn);
			        	}
			//Automatic Activation of the Compressor

			        	accelerationX = accel.getAcceleration(ADXL345_I2C.Axes.kX);
			        	accelerationY = accel.getAcceleration(ADXL345_I2C.Axes.kY);
			        	accelerationZ = accel.getAcceleration(ADXL345_I2C.Axes.kZ);
			        	accelerations = accel.getAccelerations();
			        	accelerationX = accelerations.XAxis;
			        	if (accelerationZ > 2) {
			        		first.set(true);
			        		second.set(true);
			        	}
			        	//Extends the piston when robot goes up ramp 
			        	if (accelerationZ < 1) {
			        		first.set(false);
			        		second.set(false);
			    		}
			        	//Retracts the piston when robot goes down ramp
			        	if (accelerationY > 2) {
			        		first.set(true);
			        		second.set(true);
			        	}
			        	if (accelerationY < 1) {
			        		first.set(false);
			        		second.set(false);
			    		}
			        	if (accelerationX > 2) {
			        		first.set(true);
			        		second.set(true);
			        	}
			        	if (accelerationX < 1) {
			        		first.set(false);
			        		second.set(false);
			    		}*/

		/*
		if(PSP.getRawButton(5) == true) {
			ruweUp = !ruweUp;
			Timer.delay(0.05);
		}
			if(!ruweUp){
		If Ruwe rail isn't up, turn the motor until it is up
			while(encRuwe.getDistance()<125){
			ruweRail.set(.75);
			}
			}else{
		Else, it is UP, and move the motor until it is down
			while(encRuwe.getDistance()>5){
			ruweRail.set(-.75);
		}
			}
		Activates and deactivates the Ruwe Rail

		 */

		rotate1 = SmartDashboard.getNumber("Rotate 1");
		P = SmartDashboard.getNumber("P");
		Pexp = SmartDashboard.getNumber("Pexp");
		I = SmartDashboard.getNumber("I");
		Iexp = SmartDashboard.getNumber("Iexp");
		D = SmartDashboard.getNumber("D");
		Dexp = SmartDashboard.getNumber("Dexp");
		forwardVal = SmartDashboard.getNumber("forwardVal");
		teleAngle = SmartDashboard.getNumber("teleAngle");
		SmartDashboard.putNumber("table Vals 0", tableVals[0]);
		SmartDashboard.putNumber("AUTO Counter", counter);
		SmartDashboard.putNumber("table Vals 1", tableVals[1]);
		SmartDashboard.putNumber("IR Dist 2", IRdist2);
		SmartDashboard.putNumber("Angle", angle);
		SmartDashboard.putNumber("Pressure", PSI);
		SmartDashboard.putNumber("IR Dist", IRdist);
		SmartDashboard.putNumber("IR Voltage", IR.getVoltage());
		SmartDashboard.putNumber("Rotate 1", rotate1);
		SmartDashboard.putBoolean("Facing Fowards:", facingForwards);
		SmartDashboard.putBoolean("compressor", compressor.getClosedLoopControl());
		SmartDashboard.putString("Speed Mode:", !speedMode? "Speed":"Torque");
		SmartDashboard.putNumber("forwardVal", forwardVal);
		SmartDashboard.putNumber("teleAngle", teleAngle);
		SmartDashboard.putNumber("vals0", vals[0]);
		SmartDashboard.putNumber("vals1", vals[1]);
		

	}


	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		SmartDashboard.putNumber("IR Dist 2", IRdist2);
		SmartDashboard.putNumber("Angle", angle);
		SmartDashboard.putNumber("Pressure", PSI);
		SmartDashboard.putNumber("IR Dist", IRdist);
		SmartDashboard.putNumber("Rotate 1", rotate1);
		SmartDashboard.putBoolean("Facing Fowards:", facingForwards);
		SmartDashboard.putString("Speed Mode:", !speedMode? "Speed":"Torque");
		rotate1 = SmartDashboard.getNumber("Rotate 1");
		double centerX = SmartDashboard.getNumber("centerX");

		double[] test = calcCamera(centerX);

		leftSide.setValue(test[0]);
		rightSide.setValue(test[1]);
		SmartDashboard.putNumber("test", test[2]);
	}

	/**
	 * This function determines which gear mode the robot should be using during TeleOp
	 * @param encL Left Drive Encoder
	 * @param encR Right Drive Encoder
	 * @param pd PD Board
	 * @return True for speed mode and False for torque mode
	 */
	@SuppressWarnings("deprecation")
	public boolean shiftAlgorithm(boolean currState, Encoder encL, Encoder encR, PowerDistributionPanel pd, Joystick l, Joystick r){
		double avgPeriod = (encL.getPeriod() + encR.getPeriod())/2;
		double avgCurrent = (pd.getCurrent(0)+pd.getCurrent(1)+pd.getCurrent(2)+pd.getCurrent(3))/4;
		double avgRate = 1/(60*avgPeriod);
		double ratio = avgRate/avgCurrent;

		if(ratio > 190){
			currState = true;
		}
		if(ratio < 141){
			currState = false;
		}
		return currState;
	}

}
