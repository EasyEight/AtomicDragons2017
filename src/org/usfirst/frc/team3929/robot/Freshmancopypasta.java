package org.usfirst.frc.team3929.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.networktables.*;

import edu.wpi.first.wpilibj.CameraServer;
import org.opencv.core.Mat;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;


import com.kauailabs.navx_mxp.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Freshmancopypasta extends IterativeRobot {

	
	public enum AutoState {
		START, GO, TURN, ALIGN, PIDALIGN, PLACE, FINISH
	}
	public enum TurnDirection{
		LEFT, RIGHT, STRAIGHT
	}
	AutoState CurrentAutoState = AutoState.FINISH;
	
	TurnDirection Starter = TurnDirection.STRAIGHT;

	Command autonomousCommand;
	SendableChooser<Command> chooser = new SendableChooser<>();
	//.008 is a okay so is .07
	final double kPgyro = 0.05;
	final double kIgyro = 0.0;
	final double kDgyro = 0.0;
	final double MAX_ROTATION_INPUT = .6;
	final double MINIMUM_ROTATION_INPUT = .4;
	final double MAX_VISION_INPUT = 0.5;
	
	PIDTool pidGyro, pidVision;

	//DRIVE TRAIN SPEED CONTROLLERS
	VictorSP fL;
	VictorSP fR;
	VictorSP bL;
	VictorSP bR;
	VictorSP c1;
	VictorSP c2;
	
	
	//DRIVE TRAIN ENCODERS
	Encoder leftEncoder, rightEncoder;
	int leftCount, rightCount;
	double driveDistance, dpp;
	
	double offsetFactor;
	
	//DRIVE JOYS
	Joystick joy;
	Joystick opjoy;
	Joystick haroldsjoystick;
	DigitalInput lim;
	RobotDrive drive;
	VictorSP leadScrew;
	
	//MECHANISMS
	DoubleSolenoid gearPiss;
	DoubleSolenoid lidPiss;
	
	CameraServer server;

	boolean zeroed;
	//VISION
	NetworkTable table;
	boolean found;
	String offset;
	double distance, center;
	boolean capturing;
	boolean straight;
	

	
	//AUTON
	Timer timer;
	double autoLeft, autoRight;
	int correctingCheck;
	double autonDrive;
	
	SendableChooser autoChooser;
	
	float drivePower;
	double rightDToffset;
	
	double testTime = 1;
	
	SerialPort serial_port;
	// IMU imu; // This class can be used w/nav6 and navX MXP.
	// IMUAdvanced imu; // This class can be used w/nav6 and navX MXP.
	AHRS imu; // This class can only be used w/the navX MXP.
	
	boolean zeroGyro;
	
	boolean realign;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		fL = new VictorSP(0);
		bL = new VictorSP(3);
		fR = new VictorSP(1);
		bR = new VictorSP(2);
		c1 = new VictorSP(7);
		c2 = new VictorSP(6);
		
		joy = new Joystick(0);
		opjoy = new Joystick(1);
		
		
		
		//this code gives an error : ERROR: bind() to port 1735 failed: Address already in use (TCPAcceptor.cpp:108)

		offsetFactor = 1.00; 
		straight = true;
		// haroldsjoystick = new Joystick(1);
		// haroldsmotor = new VictorSP(4);
		drive = new RobotDrive(fL, bL, fR, bR);
		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", chooser);
		drivePower = 0.4f;
		lidPiss = new DoubleSolenoid(4, 5);
		gearPiss = new DoubleSolenoid(2, 3);
		server = CameraServer.getInstance();
		
		rightEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
		leftEncoder = new Encoder(2, 3, true, Encoder.EncodingType.k4X);
		
		zeroGyro = true;
		pidGyro = new PIDTool(kPgyro, kIgyro, kDgyro, 0, -MAX_ROTATION_INPUT, MAX_ROTATION_INPUT);
		pidVision = new PIDTool(kPgyro, kIgyro, kDgyro, 0, -MAX_VISION_INPUT, MAX_VISION_INPUT);                               
        table = NetworkTable.getTable("Vision");
        
        //measured in inches. 3592 pulse average between -3897 left and 3287 right
		rightDToffset = 1.254;
		dpp = 12.54;
		
		//allowCamFront = true;

		try {

			// Use SerialPort.Port.kOnboard if connecting nav6 to Roborio Rs-232
			// port
			// Use SerialPort.Port.kMXP if connecting navX MXP to the RoboRio
			// MXP port
			// Use SerialPort.Port.kUSB if connecting nav6 or navX MXP to the
			// RoboRio USB port

			serial_port = new SerialPort(57600, SerialPort.Port.kMXP);

			// You can add a second parameter to modify the
			// update rate (in hz) from. The minimum is 4.
			// The maximum (and the default) is 100 on a nav6, 60 on a navX MXP.
			// If you need to minimize CPU load, you can set it to a
			// lower value, as shown here, depending upon your needs.
			// The recommended maximum update rate is 50Hz

			// You can also use the IMUAdvanced class for advanced
			// features on a nav6 or a navX MXP.

			// You can also use the AHRS class for advanced features on
			// a navX MXP. This offers superior performance to the
			// IMU Advanced class, and also access to 9-axis headings
			// and magnetic disturbance detection. This class also offers
			// access to altitude/barometric pressure data from a
			// navX MXP Aero.

			byte update_rate_hz = 50;
			// imu = new IMU(serial_port,update_rate_hz);
			// imu = new IMUAdvanced(serial_port,update_rate_hz);
			imu = new AHRS(serial_port, update_rate_hz);
		} catch (Exception ex) {

		}

		

	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		autonomousCommand = chooser.getSelected();
		
		CurrentAutoState = AutoState.START;

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (autonomousCommand != null)
			autonomousCommand.start();
		
		//resetting autonomous parameters
		imu.zeroYaw();
		resetEncoders();
		pidGyro.setSetpoint(0.0);
		pidVision.setSetpoint(0.0);
		realign = true;
		
		found = false;
		offset = "default";
		distance = 100;
		capturing = false;
		center = 10;
		
		//get rid of this after pid tuning

	}

	/**
	 * This function is called periodically during autonomous
	 */
	
	//Overestimating distance by 2 inches
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		lidPiss.set(DoubleSolenoid.Value.kForward);
		
		found = table.getBoolean("found", false);
		offset = table.getString("offset", "default");
		distance = table.getNumber("distance", 60.0);
		capturing = table.getBoolean("capturing", false);
		center = table.getNumber("center", 10);
		
		System.out.println("offset " + offset);
		System.out.println("distance" + distance);
		SmartDashboard.putString("DB/String 4", "Found: " + found);
		SmartDashboard.putString("DB/String 5", "Offset: " + offset);
		SmartDashboard.putString("DB/String 6", Double.toString(distance));
		SmartDashboard.putString("DB/String 1", "Drive Distance " + driveDistance);
		getEncoders();
		
		table.putNumber("offset m", 1.2);
		
		switch(CurrentAutoState) {
		case START:
			//Currently going to TURN because we haven't calibrated encoders for GO
			CurrentAutoState = AutoState.GO;
			break;
		case GO:
			if(Starter == TurnDirection.RIGHT)
				autonDrive = 80;
			else if(Starter == TurnDirection.LEFT)
				autonDrive = 80;
			else if(Starter == TurnDirection.STRAIGHT)
				autonDrive = 60;
				getEncoders();
				if((driveDistance + 18) <= autonDrive){
				autoLeft = 0.5;
				autoRight = 0.6 ;
			} else {
				autoRight = 0.0;
				autoLeft = 0.0;
				CurrentAutoState = AutoState.PLACE;
				resetEncoders();
			}
			break;
		case TURN:
			
			//found is true when the contours are within width height ratios and alignment specifications
			if(found){
				System.out.println("Tape Found? " + found);
				autoLeft = 0.0;
				autoRight = 0.0;
				CurrentAutoState = AutoState.ALIGN;
				resetEncoders();
			} else if(Starter == TurnDirection.RIGHT){
				autoLeft = 0.5;
				autoRight = -(0.5 * rightDToffset);
				}
				else if(Starter == TurnDirection.LEFT){
					autoLeft = -0.5;
					autoRight = (0.5 * dpp);
				}
				else{
					autoLeft = 0;
					autoRight = 0;
				}
			;
			break;
		case ALIGN:
			System.out.println(offset);
			if(found){
				System.out.println("Tape Found? " + found);
				if(offset.equals("left")){
					System.out.println("Offset: " + offset);
					autoLeft = -0.48;
					autoRight = 0.48 * rightDToffset;
					
				} else if(offset.equals("right")){
					System.out.println("Offset: " + offset);
					autoRight = -(0.48 * rightDToffset);
					autoLeft = 0.48;
				} else if(offset.equals("centered")){
					autoRight = 0.0;
					autoLeft = 0.0;
					resetEncoders();
					CurrentAutoState = AutoState.PLACE;
				} 
			} else {
				if(realign && autoRight > 0.0){
					autoLeft = 0.5;
					autoRight = -0.55;
					realign = false;
				}else if(realign && autoLeft > 0.0){
					autoLeft = -0.5;
					autoRight = 0.55;
					realign = false;
				}
			}
				break;
		case PIDALIGN:
			System.out.println(offset);
			if(found){
				autoLeft = pidVision.computeControl(center);
				autoRight = pidVision.computeControl(center);
				if(center<5 && -5<center){
					CurrentAutoState = AutoState.FINISH;
				}
			} else {
				if(realign && autoRight > 0.0){
					autoLeft = 0.5;
					autoRight = -(0.5 * rightDToffset);
					realign = false;
				}else if(realign && autoLeft > 0.0){
					autoLeft = -0.5;
					autoRight = 0.5 * rightDToffset;
					realign = false;
				}
			}
				break;
		case PLACE:
			System.out.println(offset);
			if(Starter == TurnDirection.STRAIGHT){
				autoLeft = 0.0;
				autoRight = 0.0;
				gearPiss.set(DoubleSolenoid.Value.kReverse);
				Timer.delay(1);
				CurrentAutoState = AutoState.FINISH;
				resetEncoders();
			}
			/*if(offset.equals("centered")){
				
				//before 75, 88
				if(64 <= distance && distance<=70){
					autoLeft = 0.0;
					autoRight = 0.0;
					gearPiss.set(DoubleSolenoid.Value.kReverse);
					Timer.delay(1);
					CurrentAutoState = AutoState.FINISH;
					resetEncoders();
//					resetEncoders();
				} else if (distance > 70.0){
					autoLeft = 0.5 ;
					autoRight = 0.5 * rightDToffset;
				}else {
					autoLeft = 0;
					autoRight = 0;
				}
			}
			else{
				CurrentAutoState = AutoState.ALIGN;
			}*/
			break;
		case FINISH:
			gearPiss.set(DoubleSolenoid.Value.kForward);
		/*	if(Math.abs(driveDistance) < 10){
				autoLeft = -0.5;
				autoRight = -0.5   * rightDToffset;
			}*/
			if(Math.abs(driveDistance) < 15){
				autoLeft = -0.58;
				autoRight = -0.6;
			} else {
				autoLeft = 0.0;
				autoRight = 0.0;
			}
			break;
			}
		drive.tankDrive(autoLeft, autoRight);
		SmartDashboard.putString("DB/String 9", Integer.toString(rightEncoder.get()));
		SmartDashboard.putString("DB/String 2", "Encoder Distance: " + driveDistance);
		SmartDashboard.putString("DB/String 1", "Current Autonomous State: " + CurrentAutoState);
		SmartDashboard.putString("DB/String 3", "Gyro Angle: " + imu.getYaw());
		SmartDashboard.putString("DB/String 4", "Found: " + found);
		SmartDashboard.putString("DB/String 5", "Offset: " + offset);
		SmartDashboard.putString("DB/String 6", "DistanceA: " + distance);
		SmartDashboard.putString("DB/String 8", "State" + CurrentAutoState);
		SmartDashboard.putString("DB/String 7", "PID "+ pidVision.computeControl(center));
		//drive.mecanumDrive_Polar(0, 0, pidGyro.computeControl(imu.getYaw()));
		//drive.tankDrive(-pidGyro.computeControl(imu.getYaw()), pidGyro.computeControl(imu.getYaw()));
		//System.out.println(pidGyro.computeControl(imu.getYaw()));
		
		System.out.println(driveDistance);
	}
	

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null)
			autonomousCommand.cancel();
		
		pidGyro.setSetpoint(0.0);
		
		CurrentAutoState = AutoState.START;
		
//		Thread t = new Thread(() -> {
//		//start Cameras
//			
//			boolean allowCamFront = true;
//		UsbCamera frontCam = CameraServer.getInstance().startAutomaticCapture(0);
//		UsbCamera backCam = CameraServer.getInstance().startAutomaticCapture(1);
//		frontCam.setResolution(320,240);
//		frontCam.setFPS(30);
//		backCam.setResolution(320, 240);
//		backCam.setFPS(30);
//		CvSink frontSink = CameraServer.getInstance().getVideo(frontCam);
//        CvSink backSink = CameraServer.getInstance().getVideo(backCam);
//        CvSource outputStream = CameraServer.getInstance().putVideo("Switcher", 320, 240);
//        
//        Mat image = new Mat();
//        
//        while(!Thread.interrupted()){
//            if(joy.getRawButton(4)) {
//        		allowCamFront = !allowCamFront;
//        	}
//        	
//            if(allowCamFront){
//              backSink.setEnabled(false);
//              frontSink.setEnabled(true);
//              frontSink.grabFrame(image);
//            } else{
//              frontSink.setEnabled(false);
//              backSink.setEnabled(true);
//              backSink.grabFrame(image);     
//            }
//            
//            outputStream.putFrame(image);
//        }
//        
//        
//		});
//		t.start();
		UsbCamera cam0 = CameraServer.getInstance().startAutomaticCapture(0);
		UsbCamera cam1 = CameraServer.getInstance().startAutomaticCapture(1);
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		
		
		distance = table.getNumber("distance", 100);
		System.out.println(found);
		getEncoders();
		table.putNumber("offset m", 1.8);
		capturing = table.getBoolean("capturing", false);
		System.out.println(capturing);
		
		if(joy.getRawButton(5)){
			drivePower = 1;
		}
		else{
			drivePower=.8f;
		}
		Scheduler.getInstance().run();
		drive.tankDrive((-joy.getRawAxis(1) * drivePower), (-joy.getRawAxis(5) * rightDToffset) * drivePower);
		
		
		//Straight driving
		if (joy.getRawButton(6)) {
			if(zeroGyro){
				imu.zeroYaw();
				zeroGyro = false;
			}
			drive.tankDrive((-joy.getRawAxis(5) - pidGyro.computeControl(imu.getYaw())) * drivePower, -joy.getRawAxis(5) * drivePower);
			
			
		}

		
		//Backwards driving
		if (joy.getRawButton(3)) {
			drive.tankDrive((joy.getRawAxis(1) * offsetFactor) * drivePower, joy.getRawAxis(5) * drivePower);
			
			//Backwards and Straight
			if (joy.getRawButton(6)) {
				if(zeroGyro){
					imu.zeroYaw();
					zeroGyro = false;
				}
				drive.tankDrive((joy.getRawAxis(1) * offsetFactor) * drivePower, joy.getRawAxis(1) * drivePower);
			}
		}
		
		
		//Pneumatics actuating
		if (opjoy.getRawButton(5)) {
			lidPiss.set(DoubleSolenoid.Value.kForward);
		} else if (opjoy.getRawButton(6)) {
			lidPiss.set(DoubleSolenoid.Value.kReverse);
		} else {
			lidPiss.set(DoubleSolenoid.Value.kOff);
		}

		if (opjoy.getRawButton(2)) {
			gearPiss.set(DoubleSolenoid.Value.kReverse);
			Timer.delay(.75);
			gearPiss.set(DoubleSolenoid.Value.kForward);
		}
		
		//Climber controls
		if(opjoy.getRawButton(3)) {
			c1.set(0.4); 
			c2.set(0.4);
		} else {
			c1.set(0); 
			c2.set(0);
		}
		
		
		//i commented this out cuz idk wot it duz
		//gearPiss.set(DoubleSolenoid.Value.kForward);
	
		

		testTime++;
		//table.putNumber("time", testTime);
		//System.out.println(testTime);
		//System.out.println("Found?" + found + "offset?" + offset + "distance" + distance);
		
		//System.out.println("Gyro: " + imu.getYaw());
		System.out.println("Left Joy: " + joy.getRawAxis(1) + "Right Joy: " + joy.getRawAxis(5));
		if(joy.getRawButton(1)){
			resetEncoders();
		} if(joy.getRawButton(2)){
			imu.zeroYaw();
		}
		
		
		
		//334 inches
		SmartDashboard.putString("DB/String 1", "Left Encoder");
		SmartDashboard.putString("DB/String 6", Double.toString(-leftEncoder.get()));
		
		SmartDashboard.putString("DB/String 2", "Right Encoder");
		SmartDashboard.putString("DB/String 7", Double.toString(rightEncoder.get()));
		
		SmartDashboard.putString("DB/String 5",  "Distance" + driveDistance);
		

		SmartDashboard.putString("DB/String 3", "Gyro Angle");
		SmartDashboard.putString("DB/String 8", Double.toString(imu.getYaw()));
		
		//System.out.println("Distance" + distance );
		
		found = table.getBoolean("found", false);
		offset = table.getString("offset", "default");
		distance = table.getNumber("distance", 60.0);
		capturing = table.getBoolean("capturing", false);
		center = table.getNumber("center", 10);
		
		System.out.println("offset " + offset);
		System.out.println("distance" + distance);
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
		found = table.getBoolean("found", false);
		offset = table.getString("offset", "default");
		distance = table.getNumber("distance", 100.0);
		capturing = table.getBoolean("capturing", false);
		
		SmartDashboard.putString("DB/String 4", "Found: " + found);
		SmartDashboard.putString("DB/String 5", "Offset: " + offset);
		SmartDashboard.putString("DB/String 6", "Distance Away: " + distance);
		SmartDashboard.putString("DB/String 1", "Capturing: " + capturing);
	}
	public void getEncoders(){
		leftCount = -leftEncoder.get();
		rightCount = rightEncoder.get();
		
		driveDistance = ((leftCount + rightCount)/2)/dpp;
	}
	public void resetEncoders(){
		leftCount = 0;
		rightCount = 0;
		driveDistance = 0;
		
		leftEncoder.reset();
		rightEncoder.reset();
	}
	public void startCameras(){
        

		
	}
}