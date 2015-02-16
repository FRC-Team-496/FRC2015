package org.usfirst.frc.team496.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the RobotDrive class. The
 * SampleRobot class is the base of a robot application that will automatically
 * call your Autonomous and OperatorControl methods at the right time as
 * controlled by the switches on the driver station or the field controls.
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SampleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 *
 * WARNING: While it may look like a good choice to use for your code if you're
 * inexperienced, don't. Unless you know what you are doing, complex code will
 * be much more difficult under this system. Use IterativeRobot or Command-Based
 * instead if you're new.
 */
public class Robot extends SampleRobot {
	RobotDrive myRobot;
	Joystick stick, winchStick;
	Talon frontRight, frontLeft, rearRight, rearLeft;
	Victor winch;
	Gyro gyro;
	double x, y, twist, a, JoyX, JoyY, JoyTwist;
	Compressor compressor;
	DoubleSolenoid brake, grabber;
	DigitalInput topLimit, bottomLimit;
	PowerDistributionPanel pdp;

	public Robot() {
		frontRight = new Talon(3);
		frontLeft = new Talon(2);
		rearRight = new Talon(1);
		rearLeft = new Talon(0);
		myRobot = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);
		myRobot.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
		myRobot.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
		myRobot.setExpiration(0.1);
		stick = new Joystick(0);
		winchStick = new Joystick(1);
		winch = new Victor(4);
		gyro = new Gyro(0);
		pdp = new PowerDistributionPanel();

		compressor = new Compressor(0);
		grabber = new DoubleSolenoid(6, 7);
		brake = new DoubleSolenoid(0, 1);
		topLimit = new DigitalInput(0); 
		bottomLimit = new DigitalInput(1);
		
				
		grabber.set(DoubleSolenoid.Value.kReverse);
		brake.set(DoubleSolenoid.Value.kReverse);
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	public void autonomous() {
		grabber.set(DoubleSolenoid.Value.kReverse);
		brake.set(DoubleSolenoid.Value.kReverse);
		myRobot.setSafetyEnabled(false);
		
		
		winch.set(-0.6); //negative is up
		Timer.delay(2);
		winch.set(0);
		brake.set(DoubleSolenoid.Value.kForward);
		myRobot.mecanumDrive_Cartesian(0, -0.4, 0, 0); //negative is forward
		Timer.delay(0.8);
		myRobot.mecanumDrive_Cartesian(0, 0, 0, 0);
		brake.set(DoubleSolenoid.Value.kReverse);
		Timer.delay(3);
		grabber.set(DoubleSolenoid.Value.kForward);
		Timer.delay(1);
		winch.set(-0.6);
		Timer.delay(1);
		brake.set(DoubleSolenoid.Value.kForward);
		winch.set(0);
		Timer.delay(0.2);
		myRobot.mecanumDrive_Cartesian(0, 0, -0.32, 0); //negative rotates left
		Timer.delay(2);
		myRobot.mecanumDrive_Cartesian(0, 0, 0, 0);
		myRobot.mecanumDrive_Cartesian(0, -0.5, 0, 0);
		Timer.delay(3);
		myRobot.mecanumDrive_Cartesian(0, 0, 0, 0);
		
		
	}

	public void operatorControl() {
		myRobot.setSafetyEnabled(true);
		boolean isBraked = false;
		boolean isLatched = false;
		grabber.set(DoubleSolenoid.Value.kReverse);
		brake.set(DoubleSolenoid.Value.kReverse); 

		while (isOperatorControl() && isEnabled()) {
			//power();
			SmartDashboard.putNumber("Voltage", pdp.getVoltage());
			SmartDashboard.putNumber("Current", pdp.getTotalCurrent());
			SmartDashboard.putNumber("winch motor 1", pdp.getCurrent(14));
			SmartDashboard.putNumber("winch motor 2", pdp.getCurrent(15));
			SmartDashboard.putBoolean("Upper Switch", topLimit.get());
			SmartDashboard.putBoolean("BOttom Switch", bottomLimit.get());
			SmartDashboard.putBoolean("Brake", isBraked);
			SmartDashboard.putNumber("Winch Stick", winchStick.getY());
			SmartDashboard.putNumber("Winch Value", winch.get());
			SmartDashboard.putBoolean("Latched", isLatched);
			x = stick.getX();
			y = stick.getY();
			twist = stick.getTwist();
			a = (stick.getThrottle() + 1) / 2;

			// System.out.println("a value: "+a);

			JoyX = a * Math.pow(x, 3) + (1 - a) * x;
			JoyY = a * Math.pow(y, 3) + (1 - a) * y;
			//JoyTwist = a * Math.pow(twist, 3) + (1 - a) * twist;
			JoyTwist = twist/2;

			//System.out.println("gyro: " + gyro.getAngle());
			myRobot.mecanumDrive_Cartesian(JoyX, JoyY, JoyTwist, 0);
			// myRobot.arcadeDrive(stick); // drive with arcade style (use right
			// stick)
			Timer.delay(0.005); // wait for a motor update time

			if (stick.getRawButton(1) == true) {
				grabber.set(DoubleSolenoid.Value.kForward);
			}

			if (stick.getRawButton(2) == true) {
				grabber.set(DoubleSolenoid.Value.kReverse);
			}

			if (winchStick.getRawButton(1) == true) {
				brake.set(DoubleSolenoid.Value.kForward);
				isBraked = true;
			}

			if (winchStick.getRawButton(2) == true) {
				brake.set(DoubleSolenoid.Value.kReverse);
				isBraked = false;
			}
			
			
			boolean isTop = topLimit.get();
			boolean isBottom = bottomLimit.get();

			if (isTop && winchStick.getY() < 0 ) {
				winch.set(0);
				isLatched = true;
				brake.set(DoubleSolenoid.Value.kForward);
				isBraked = true;
			}
			else if (isBottom && winchStick.getY() > 0){
				winch.set(0);
			} else if (isBraked) {
				winch.set(0);
			} else if (isLatched) {
				if (winchStick.getY() < 0) {
					winch.set(0);
				} else if(winchStick.getY() >= 0) {
					isLatched = false;
				}
			} else {
				winch.set(winchStick.getY());
			}
			
			//System.out.println("upper limit switch: " + topLimit.get());
			SmartDashboard.putNumber("winch motor 1", pdp.getCurrent(14));
			SmartDashboard.putNumber("winch motor 2", pdp.getCurrent(15));
		}
	}

	/**
	 * Runs during test mode
	 */
	public void test() {
	}
	
	public void power() {
		SmartDashboard.putNumber("Voltage", pdp.getVoltage());
		SmartDashboard.putNumber("Current", pdp.getTotalCurrent());
		
	}
}
