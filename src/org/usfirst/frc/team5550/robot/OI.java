package org.usfirst.frc.team5550.robot;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

	//Xbox Controller Inputs only change after = sign to ControllerIO.<buttonName>
	public static final int angleButton = ControllerIO.b;
	public static final int armPos1Button = ControllerIO.lb;
	public static final int armPos2Button = ControllerIO.rb;
	
	public static final int switchCamButton = 11;
	


	
	
	public static long timeClick = 0;
}

