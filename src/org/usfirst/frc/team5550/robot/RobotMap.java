package org.usfirst.frc.team5550.robot;
/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	
	//Set motor to color using Motor.colorName only change things after the = sign
	//Yellow blue is yellow_blue
	public static final int leftDrive = Motor.blue;
	public static final int rightDrive = Motor.red;
	public static final int ballPositioner = Motor.yellow_blue;
	public static final int ballLauncher = Motor.black;
	public static final int armWheels = Motor.green;
	public static final int lift = Motor.yellow;
	
	//Max percentage speed of drive speed at different buttons 7-12 on Logitech Joystick
	//Button 7 is default speed
	public static double button7 = 0.50;
	public static double button8 = 0.75;
	public static double button9 = 1.0;
	
	public static double armWheelSpeed = 1;
	public static double armPower = 1;
	
	public static double liftSpeed = 0.5;
	
	public static int directionProfile = 1;
	
	//Divide percentage value of launch speed by 100. E.g. 87/100 = 0.87
	//Only edit after = sign
	//If motors are going the wrong way make the value the opposite sign E.g. 1.0 -> -1.0
	public static double launcherSpeedPercent = 1.00;
	public static double positionerSpeed = 1.00;
	
	public static double liftStartTime = 135;
	
	public static double driveSpeedPercent = button7;
}

