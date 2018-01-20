package org.usfirst.frc.team2990.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Relay;

import org.usfirst.frc.team2990.robot.DriveTrain;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends IterativeRobot {

	float lerpSpeed = 0.2f;
	public AHRS navx = new AHRS(SPI.Port.kMXP);
	public DriveTrain drivetrain = new DriveTrain(3, 5, 6, 1, 2, 4, navx);
	//public JoshMotorControllor climber= new JoshMotorControllor(7, lerpSpeed,false);
	public DoubleSolenoid driveShifter = new DoubleSolenoid(0,1);
	public Joystick xbox360Controller;
	public Joystick xboxController;
	public Ultrasonic ultrasonic = new Ultrasonic(5, 6);
	public Encoder encoder;

	float speedo = 0.2f; //TODO speed for PID DriveStraight

	public AutoStep step = new AutoStep(drivetrain, navx, encoder);
	public AutoStep[] autoTest = new AutoStep[1];
	public int currentStep = 0;

	public void robotInit()
	{
		encoder = new Encoder(3,4);
		encoder.reset();

	}

	public void autonomousInit() {
		driveShifter.set(DoubleSolenoid.Value.kReverse);
		navx.reset();
		encoder.reset();

		autoTest[0] = new AutoStep(drivetrain, navx, encoder);
		autoTest[0].MoveForward(10000.0f, 1.0f);
		currentStep = 0;
	}

	public void autonomousPeriodic() {

		LogInfo("AUTO");
		driveShifter.set(DoubleSolenoid.Value.kReverse);
		//step.Update();
		UpdateMotors();
		LogInfo("AutoTest[" + currentStep + "]");
		
		if (currentStep < autoTest.length){
			autoTest[currentStep].Update();
			if (autoTest[currentStep].isDone) {
				currentStep++;
				if (currentStep < autoTest.length) {
					autoTest[currentStep].InitStep();
				}
			}
		}
	}

	public void teleopInit() {

		LogInfo("TELEOP");

		ultrasonic.setAutomaticMode(true);

		xbox360Controller = new Joystick(0);
		xboxController = new Joystick(1);


		float lerpSpeed = 0.5f;
		drivetrain.SetLeftSpeed(lerpSpeed);
		drivetrain.SetRightSpeed(lerpSpeed);
	}

	public void teleopPeriodic() {
		//LogInfo("Encoder: " + encoder.get());
		//LogInfo("Ultrasonic: " + ultrasonic.getRangeInches());
		//LogInfo("Navx: " + navx.getYaw());

		UpdateMotors();
		ControllerDrive();

	}

	public void testInit() {
		xbox360Controller = new Joystick(0);
		xboxController = new Joystick(1);
		float lerpSpeed = 0.5f;
		drivetrain.SetLeftSpeed(lerpSpeed);
		drivetrain.SetRightSpeed(lerpSpeed);
		/*
		SmartDashboard.putNumber("P: ", turnController.getP());
		SmartDashboard.putNumber("I: ", turnController.getI());
		SmartDashboard.putNumber("D: ", turnController.getD());
		SmartDashboard.putNumber("F: ", turnController.getF());
		*/
		SmartDashboard.putNumber("Speed: ", speedo);
	}
	public void testPeriodic() {
		if (xbox360Controller.getRawButton(4)) {
			drivetrain.DriveStraight(speedo, false);
		} else {
			ControllerDrive();
		}
		if (xbox360Controller.getRawButton(10)) {
			navx.reset();
		}
		UpdateMotors();
		SmartDashboard.putNumber("NavX: ", navx.getYaw());
		SmartDashboard.putNumber("Encoder: ", encoder.getDistance());
		SmartDashboard.putNumber("Ultrasonic: ", ultrasonic.getRangeInches());
		/*turnController.setP(SmartDashboard.getNumber("P: ", turnController.getP()));
		turnController.setI(SmartDashboard.getNumber("I: ", turnController.getI()));
		turnController.setD(SmartDashboard.getNumber("D: ", turnController.getD()));
		turnController.setF(SmartDashboard.getNumber("F: ", turnController.getF()));*/
		speedo = (float) SmartDashboard.getNumber("Speed: ", 0.2f);
		System.out.print("Encoders: "+ encoder.getDistance() );


	}

	public void UpdateMotors() {
		drivetrain.Update();

	}
	public float TranslateController(float input) {
		float deadzone = 0.15f;
		if (input > -deadzone && input < deadzone) {
			input = 0.0f;
		}
		float a = 0.7f;
		float output = (a * input * input * input) + (1 - a) * input; 
		return output;
	}


	//custom classes
	public void LogInfo(String info) {
		System.out.println(info + ";    ");
	}

	public void ControllerDrive()
	{
		float horJoystick = TranslateController((float)xbox360Controller.getRawAxis(0));
		float verJoystick = TranslateController((float)xbox360Controller.getRawAxis(5));

		drivetrain.SetRightSpeed(verJoystick + horJoystick);
		drivetrain.SetLeftSpeed(-verJoystick + horJoystick);

		if (xbox360Controller.getRawButton(5)) {
			driveShifter.set(DoubleSolenoid.Value.kReverse);
		}
		if (xbox360Controller.getRawButton(6)) {
			driveShifter.set(DoubleSolenoid.Value.kForward);
		}
	}
}
