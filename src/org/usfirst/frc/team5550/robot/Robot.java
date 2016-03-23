
package org.usfirst.frc.team5550.robot;

import org.usfirst.frc.team5550.robot.commands.ExampleCommand;
import org.usfirst.frc.team5550.robot.subsystems.ExampleSubsystem;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.RumbleType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.USBCamera;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();

	Command autonomousCommand;
	SendableChooser chooser;
	Joystick joystick0;
	Joystick controller0;

	// Motor type may change
	TalonSRX leftDrive;
	TalonSRX rightDrive;
	// Talon ballRetriever;
	Talon launcher;
	Talon lift;
	VictorSP ballPositioner;
	CANTalon ballArm;
	VictorSP armWheels;
	DigitalInput ballCapture;
	// VictorSP lift2;
	Ultrasonic us;
	AnalogGyro gyro;
	CameraServer server;
	Preferences prefs;

	double angle = 0;

	double armInitPos = 0;
	double armPos = 0;

	int autoID;
	boolean autoDone = false;
	double shootBallStart = 0;

	boolean reachedWall = false;
	boolean doneTurning = false;

	USBCamera cam0, cam1;
	boolean curCam;
	Image img = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		chooser = new SendableChooser();
		chooser.addDefault("Default Auto", new ExampleCommand());
		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", chooser);

		// Joystick Initialization Code
		joystick0 = new Joystick(0);
		controller0 = new Joystick(1);
		OI.timeClick = System.currentTimeMillis();

		// Motor Initialization Code
		launcher = new Talon(RobotMap.ballLauncher);
		ballPositioner = new VictorSP(RobotMap.ballPositioner);
		leftDrive = new TalonSRX(RobotMap.leftDrive);
		rightDrive = new TalonSRX(RobotMap.rightDrive);
		armWheels = new VictorSP(RobotMap.armWheels);
		// ballRetriever = new Talon(RobotMap.ballRetriever);
		lift = new Talon(RobotMap.lift);
		ballArm = new CANTalon(0);
		ballArm.changeControlMode(CANTalon.TalonControlMode.PercentVbus);

		ballCapture = new DigitalInput(9);

		// ballArm.reverseSensor(true);
		// ballArm.setPID(0.5, 1, 1);

		us = new Ultrasonic(1, 2);
		gyro = new AnalogGyro(0, 1900694, 0.272727);
		LiveWindow.addSensor("Sensors", "Ultrasonic", us);
		gyro.setSensitivity(0.0011);
		// Expiration Code
		launcher.setExpiration(0.1);
		ballPositioner.setExpiration(0.1);
		leftDrive.setExpiration(0.1);
		rightDrive.setExpiration(0.1);
		us.setAutomaticMode(true);
		us.startLiveWindowMode();
		ballArm.setExpiration(0.1);
		armWheels.setExpiration(0.1);

		// cam0 = new USBCamera("cam0");
		// cam1 = new USBCamera("cam1");

		server = CameraServer.getInstance();
		server.setQuality(50);

		prefs = Preferences.getInstance();
		// ballRetriever.setExpiration(0.1);
		// lift.setExpiration(0.1);
		// Dashboard Initialization Code
		updateDashboard();
	}

	public double getCurrentTime() {
		return DriverStation.getInstance().getMatchTime();
	}

	public void updateDashboard() {
		SmartDashboard.putNumber("GyroAngle", convert(gyro.getAngle()));
		SmartDashboard.putNumber("GyroCenter", gyro.getCenter());
		SmartDashboard.putNumber("GyroOffset", gyro.getOffset());

		SmartDashboard.putNumber("BallArmPosition", ballArm.getPosition() - armInitPos);

		SmartDashboard.putData("DriveLeftController", leftDrive);
		SmartDashboard.putData("DriveRightController", rightDrive);

		SmartDashboard.putData("ShooterController", launcher);
		SmartDashboard.putData("PositionerController", ballPositioner);

		SmartDashboard.putData("LiftController", lift);

		SmartDashboard.putData("BallArmController", ballArm);
		SmartDashboard.putData("RetrieverWheelsController", armWheels);

		SmartDashboard.putBoolean("Camera", curCam);

	}

	public void initCamera() {
		curCam = ((prefs.getInt("Camera", 0) == 0) ? false : true);
		cam0.openCamera();
		cam1.openCamera();
		cam0.stopCapture();
		cam1.stopCapture();
		System.out.println("Camera init");
		(curCam ? cam1 : cam0).startCapture();
		try {
			(curCam ? cam1 : cam0).getImage(img);
			server.setImage(img);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	public void updateCamera() {
		try {
			(curCam ? cam1 : cam0).getImage(img);
			server.setImage(img);
		} catch (Exception e) {
			e.printStackTrace();
			System.out.println("Failed to set image to server");
		}
	}

	public void updateArmPos() {
		try {
			armPos = ballArm.getPosition();
		} catch (Exception e) {
		}
	}

	public void drive(double throttle, double turn) {
		throttle = throttle * RobotMap.driveSpeedPercent;
		double t_left = throttle + turn;
		double t_right = throttle - turn;
		leftDrive.set(t_left);
		rightDrive.set(t_right);
	}

	public float convert(float angle) {
		return ((float) angle * (480 / 360));
	}

	public double convert(double angle) {
		return angle * (480 / 360);
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	public void disabledInit() {
		try {
			cam0.closeCamera();
			cam1.closeCamera();
		} catch (Exception e) {

		}
		controller0.setRumble(RumbleType.kLeftRumble, 0f);
		controller0.setRumble(RumbleType.kRightRumble, 0f);
	}

	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	public void autonomousInit() {
		gyro.reset();
		reachedWall = false;
		doneTurning = false;
		initCamera();
		autoID = prefs.getInt("AutoID", 1);
	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {
		angle = convert(gyro.getAngle());
		boolean shootBall = prefs.getBoolean("BallShootAuto", false);
		double range = us.getRangeInches();
		double time = getCurrentTime();
		double launcherAutoSpeed = prefs.getDouble("LauncherAutoSpeed", 1);
		float speed = prefs.getFloat("AutoSpeed", 0.3F);
		float maxRange = prefs.getFloat("Max1", 0.0f);
		float minRange = prefs.getFloat("Min1", 0.0F);
		float desiredAngle = convert(prefs.getFloat("TurnAngle1", 0.0f));
		if (!autoDone) {
			switch (autoID) {
			case 1:
				if (range > minRange + 1 && !reachedWall) {
					if (range < maxRange && range > minRange) {
						speed = speed * (float) (range - minRange) / (maxRange - minRange);
					}
					if (range > minRange) {
						driveStraight(speed, angle);
						// System.out.println(angle);
						// System.out.println(range);
					} else {
						rightDrive.set(0);
						leftDrive.set(0);
					}
				} else if (!doneTurning) {
					System.out.println((desiredAngle - angle) + " " + (Math.abs(desiredAngle - angle) > 10));
					if (Math.abs(desiredAngle - angle) > 10) {
						drive(0, -0.5);
						reachedWall = true;
						System.out.println("Reached First Wall");
					} else {
						drive(0, 0);
						doneTurning = true;
						System.out.println("Done Turning");
					}
				} else if (doneTurning && reachedWall) {
					if (range < maxRange && range > minRange) {
						speed = speed * (float) (range - minRange) / (maxRange - minRange);
					}
					if (range > minRange) {
						driveStraight(speed, angle - desiredAngle);
						// System.out.println(angle);
						// System.out.println(range);
					} else {
						drive(0, 0);
						autoDone = true;
						shootBallStart = time;
					}
				}

			case 2:
				if (time < 0.6) {
					driveStraight(0.8, angle);
				} else if (time >= 0.6 && time < 1.1) {
					driveStraight(-0.8, angle);
				} else if (time >= 1 && time < 6) {
					driveStraight(speed, angle);
				} else if (time >= 6 && range < maxRange && range > minRange) {
					speed = speed * (float) (range - minRange) / (maxRange - minRange);
				}
				if (time >= 6 && range > minRange) {
					driveStraight(speed, angle);
				} else {
					drive(0, 0);
					autoDone = true;
					shootBallStart = time;
				}
			case 3:
				if (time < 7) {
					driveStraight(speed, angle);
				} else if (time >= 7 && range < maxRange && range > minRange) {
					speed = speed * (float) (range - minRange) / (maxRange - minRange);
				}
				if (time >= 7 && range > minRange) {
					driveStraight(speed, angle);
				} else {
					drive(0, 0);
					autoDone = true;
					shootBallStart = time;
				}

			default:

			}
		}

		if (shootBall && autoDone) {
			if(shootBallStart + 3 > time) {
				launcher.set(launcherAutoSpeed);
			} else if(shootBallStart + 3 <= time && time <= shootBallStart + 4) {
				ballPositioner.set(1);
			} else {
				launcher.set(0);
				ballPositioner.set(0);
			}
		}

		updateDashboard();
		updateCamera();
		Scheduler.getInstance().run();
	}

	public void driveStraight(double speed, double angle) {
		drive(-1.0 * speed, angle * 0.015);
	}

	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null)
			autonomousCommand.cancel();
		gyro.reset();
		updateArmPos();
		armInitPos = armPos;
		System.out.println("Init Pos: " + armInitPos);

		RobotMap.button7 = prefs.getDouble("DefaultDriveSpeed", 0.5);
		RobotMap.button8 = prefs.getDouble("TurboDriveSpeed", 0.75);
		RobotMap.button9 = prefs.getDouble("UberTurboDriveSpeed", 1.0);
		RobotMap.armWheelSpeed = prefs.getDouble("MaxArmSpeed", 1.0);
		RobotMap.armPower = prefs.getDouble("ArmPower", 0.2);
		// initCamera();
		RobotMap.directionProfile = (curCam ? -1 : 1);
		RobotMap.driveSpeedPercent = RobotMap.button7;
		RobotMap.liftSpeed = prefs.getDouble("LiftSpeed", 0.5);
		RobotMap.launcherSpeedPercent = prefs.getDouble("LauncherSpeed", 1.0);
		RobotMap.positionerSpeed = prefs.getDouble("PositionerSpeed", 1.0);

		RobotMap.liftStartTime = 150 - (prefs.getDouble("LiftStartTime", 30));
		// NIVision.IMAQdxStartAcquisition(curCam);
	}

	/*
	 * public double skim(double v) { double gain = 0.3; if (v > 1.0) { return
	 * -((v - 1.0) * gain); } else if (v < -1.0) { return -((v + 1.0) * gain); }
	 * return 0; }
	 */

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {

		// drive.arcadeDrive(joystick0);
		drive(joystick0.getY() * RobotMap.directionProfile, joystick0.getX());
		// double left = t_left + skim(t_right);
		// double right = t_right + skim(t_left);
		// long currentTime = System.currentTimeMillis();

		updateArmPos();
		/*
		 * armOffBy = (armPos - armInitPos) - armDestination; double speed =
		 * Math.abs(armOffBy / armDestination) * prefs.getDouble("ArmPower", 0);
		 * if (armOffBy != 0 && Math.abs(armOffBy / armDestination) >
		 * prefs.getDouble("ArmVariabilityPercent", 0.2)) { if (armOffBy > 0) {
		 * ballArm.set(-speed); } else { ballArm.set(speed); } //
		 * ballArm.set(armOffBy * .005); } else { ballArm.set(0); }
		 * 
		 */
		if (joystick0.getRawButton(7)) {
			RobotMap.driveSpeedPercent = RobotMap.button7;
		} else if (joystick0.getRawButton(8)) {
			RobotMap.driveSpeedPercent = RobotMap.button8;
		} else if (joystick0.getRawButton(9)) {
			RobotMap.driveSpeedPercent = RobotMap.button9;
		}

		if (controller0.getRawAxis(2) > 0 || joystick0.getRawButton(5)) {
			armWheels.set(RobotMap.armWheelSpeed);
		} else if (controller0.getRawAxis(3) > 0 || joystick0.getRawButton(3)) {
			armWheels.set(-RobotMap.armWheelSpeed);
		} else {
			armWheels.set(0);
		}

		if (joystick0.getRawButton(6) || controller0.getRawButton(ControllerIO.lb)) {
			ballArm.set(-RobotMap.armPower);
		} else if (joystick0.getRawButton(4) || controller0.getRawButton(ControllerIO.rb)) {
			ballArm.set(RobotMap.armPower);
		} else {
			ballArm.set(0);
		}

		if (!ballCapture.get()) {
			controller0.setRumble(RumbleType.kLeftRumble, 0.5f);
			controller0.setRumble(RumbleType.kRightRumble, 0.5f);
		} else {
			controller0.setRumble(RumbleType.kLeftRumble, 0f);
			controller0.setRumble(RumbleType.kRightRumble, 0f);
		}

		if (controller0.getRawAxis(5) > 0.05) {
			ballPositioner.set(RobotMap.positionerSpeed);
		} else {
			ballPositioner.set(0);
		}

		if (controller0.getRawButton(ControllerIO.a)) {
			launcher.set(RobotMap.launcherSpeedPercent);
		} else if (controller0.getRawButton(ControllerIO.b)) {
			launcher.set(0);
		}

		if (joystick0.getRawButton(12)) {
			RobotMap.directionProfile = -RobotMap.directionProfile;
		}

		if (joystick0.getRawButton(OI.switchCamButton) || controller0.getRawButton(ControllerIO.back)) {
			(curCam ? cam1 : cam0).stopCapture();
			(curCam ? cam0 : cam1).startCapture();
			RobotMap.directionProfile = (curCam ? 1 : -1);
			curCam = !curCam;
		}

		/*
		 * if (controller0.getRawButton(OI.resetButton)) { gyro.reset(); } if
		 * (controller0.getRawButton(OI.calibrateButton)) { gyro.calibrate(); }
		 */
		if (getCurrentTime() > RobotMap.liftStartTime) {
			if (controller0.getRawButton(ControllerIO.x)) {
				lift.set(RobotMap.liftSpeed);
			} else if (controller0.getRawButton(ControllerIO.y)) {
				lift.set(-RobotMap.liftSpeed);
			} else {
				lift.set(0);
			}
		}

		if (joystick0.getRawButton(1)) {
			drive(0.25 * RobotMap.directionProfile, 0);
		}

		if (joystick0.getRawButton(2)) {
			drive(-0.25 * RobotMap.directionProfile, 0);
		}

		// ballRetriever.set(controller0.getRawAxis(5));

		// controller0.setRumble(RumbleType.kLeftRumble, 1F);
		// controller0.setRumble(RumbleType.kRightRumble, 1F);

		/*
		 * double ballPositionerSpeed = 1 - ((joystick0.getRawAxis(3) + 1)/2);
		 * ballPositioner.set(ballPositionerSpeed);
		 */

		// double ballRetrieverSpeed =
		// joystick0.getRawButton(OI.ballRetrievalButton) ?
		// RobotMap.retrieverSpeed : 0;
		// ballRetriever.set(ballRetrieverSpeed);
		updateDashboard();
		// TODO updateCamera();
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		updateDashboard();
	}
}
