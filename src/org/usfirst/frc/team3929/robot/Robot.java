
package org.usfirst.frc.team3929.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.networktables.*;

import org.usfirst.frc.team3929.robot.commands.ExampleCommand;
import org.usfirst.frc.team3929.robot.subsystems.ExampleSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
	public static OI oi;
	
	public enum AutoState {
		START, GO, TURN, ALIGN, PLACE, FINISH
	}
	public enum TurnDirection{
		LEFT, RIGHT, STRAIGHT
	}
	AutoState CurrentAutoState = AutoState.START;
	
	TurnDirection Starter = TurnDirection.RIGHT;

	Command autonomousCommand;
	SendableChooser<Command> chooser = new SendableChooser<>();
	
	//DRIVE TRAIN SPEED CONTROLLERS
	VictorSP fL;
	VictorSP fR;
	VictorSP bL;
	VictorSP bR;
	
	//DRIVE JOYS
	Joystick joy;
	Joystick haroldsjoystick;
	DigitalInput lim;
	RobotDrive drive;
	VictorSP leadScrew;
	
	//MECHANISMS
	DoubleSolenoid sully;
	DoubleSolenoid bully;
	
	CameraServer server;
	
	//VISION
	NetworkTable table;
	boolean found;
	String offset;
	double distance;
	
	boolean straight;
	
	//AUTON
	Timer timer;
	double autoLeft, autoRight;
	int correctingCheck;

	float drivePower;
	
	double testTime = 1;


	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		oi = new OI();
		fL = new VictorSP(1);
		bL = new VictorSP(0);
		fR = new VictorSP(3);
		bR = new VictorSP(2);
		joy = new Joystick(0);
		leadScrew = new VictorSP(4);
		straight = true;
		// haroldsjoystick = new Joystick(1);
		// haroldsmotor = new VictorSP(4);
		drive = new RobotDrive(fL, bL, fR, bR);
		chooser.addDefault("Default Auto", new ExampleCommand());
		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", chooser);
		drivePower = 0.5f;
		sully = new DoubleSolenoid(0, 1);
		bully = new DoubleSolenoid(2, 3);
		server = CameraServer.getInstance();
		
		table = NetworkTable.getTable("VisionTable");
		

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

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (autonomousCommand != null)
			autonomousCommand.start();
		

		
	}

	/**
	 * This function is called periodically during autonomous
	 */
	
	//Overestimating distance by 2 inches
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		// if(swit.get()){

		/*
		 * } else{ motor.set(0); }
		 */
		
		found = table.getBoolean("found", false);
		offset = table.getString("offset", "default");
		distance = table.getNumber("distance", 100.0);
		
		switch(CurrentAutoState) {
		case START:
			//Currently going to TURN because we haven't calibrated encoders for GO
			CurrentAutoState = AutoState.TURN;
			break;
		case GO:
/*			if(increment <=190) {
				tank.tankDrive(0.8, 0.8);
			} else {
				tank.tankDrive(0.0,0.0);
				increment = 0;
				currentAutonState = AutonState.FINISHED;
			}*/
			break;
		case TURN:
			
			//found is true when the contours are within width height ratios and alignment specifications
			if(found){
				System.out.println("Tape Found? " + found);
				autoLeft = 0.0;
				autoRight = 0.0;
				CurrentAutoState = AutoState.ALIGN;
			} else {
				if(Starter == TurnDirection.RIGHT){
				autoLeft = 0.4;
				autoRight = -0.4;
				}
				else if(Starter == TurnDirection.LEFT){
					autoLeft = -0.4;
					autoRight = 0.4;
				}
				else{
					autoLeft = 0;
					autoRight = 0;
				}
			}
			break;
		case ALIGN:
			System.out.println(offset);
			if(found){
				System.out.println("Tape Found? " + found);
				if(offset.equals("left")){
					System.out.println("Offset: " + offset);
					autoLeft = -0.4;
					autoRight = 0.4;
					
				} else if(offset.equals("right")){
					System.out.println("Offset: " + offset);
					autoRight = -0.4;
					autoLeft = 0.4;
				} else if(offset.equals("centered")){
					autoRight = 0.0;
					autoLeft = 0.0;
					CurrentAutoState = AutoState.PLACE;
				} System.out.println(autoLeft); System.out.println(autoRight);
			} else {
				/*if(i == 1 && autoRight != 0.0){
					autoLeft = 0.4;
					autoRight = 0.0;
					i++;
				}else if(i==1 && autoLeft != 0.0){
					autoLeft = 0.0;
					autoRight = 0.4;
					i++;
				}*/
				CurrentAutoState = AutoState.TURN;
				}
				break;
		case PLACE:
			System.out.println(offset);
			if(offset.equals("centered")){
				if(34 <= distance && distance<= 38){
					autoLeft = 0.0;
					autoRight = 0.0;
					sully.set(DoubleSolenoid.Value.kForward);
					CurrentAutoState = AutoState.FINISH;
				} else if (distance > 34.0){
					autoLeft = 0.4;
					autoRight = 0.4;
				}else {
					autoLeft = 0;
					autoRight = 0;
				}
			}
			else{
				CurrentAutoState = AutoState.ALIGN;
			}
			break;
		case FINISH:
			sully.set(DoubleSolenoid.Value.kOff);
			break;
			}
		drive.tankDrive(autoLeft, autoRight);
		System.out.println("Distance Away: " + distance);
		System.out.println("Current Autonomous State: " + CurrentAutoState);
	}
	

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null)
			autonomousCommand.cancel();
		
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		if(joy.getRawButton(5)){
			drivePower = 1;
		}
		else{
			drivePower=.8f;
		}
		Scheduler.getInstance().run();
		drive.tankDrive(-joy.getRawAxis(1) * drivePower, -joy.getRawAxis(5) * drivePower);
		if (joy.getRawButton(6)) {
			drive.tankDrive(-joy.getRawAxis(1) * drivePower, -joy.getRawAxis(1) * drivePower);
		}
		if (joy.getRawButton(3)) {
			drive.tankDrive(joy.getRawAxis(1) * drivePower, joy.getRawAxis(5) * drivePower);
			if (joy.getRawButton(6)) {
				drive.tankDrive(joy.getRawAxis(1) * drivePower, joy.getRawAxis(1) * drivePower);
			}
		}
		if (joy.getRawButton(1)) {
			sully.set(DoubleSolenoid.Value.kForward);
		} else if (joy.getRawButton(2)) {
			sully.set(DoubleSolenoid.Value.kReverse);
		} else {
			sully.set(DoubleSolenoid.Value.kOff);
		}
		if (joy.getRawButton(3)) {
			bully.set(DoubleSolenoid.Value.kForward);
		} else if (joy.getRawButton(4)) {
			bully.set(DoubleSolenoid.Value.kOff);
		}
	
		
	//	bully.set(DoubleSolenoid.Value.kForward);
		if (joy.getRawAxis(4) > 0) {
			leadScrew.set(joy.getRawAxis(4));
		} else if (joy.getRawAxis(3) > 0) {
			leadScrew.set(-joy.getRawAxis(3));
		} else {
			leadScrew.set(0);
		}
		
		found = table.getBoolean("found", false);
		offset = table.getString("centered", "default");
		distance = table.getNumber("distance", 100.0);
		

		testTime++;
		table.putNumber("time", testTime);
		System.out.println(testTime);
		System.out.println("Found?" + found + "offset?" + offset + "distance" + distance);

	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}