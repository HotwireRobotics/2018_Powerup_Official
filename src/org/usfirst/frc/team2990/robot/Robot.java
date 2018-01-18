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


public class Robot extends IterativeRobot implements PIDOutput {

	float lerpSpeed = 0.2f;
	public DriveTrain drivetrain = new DriveTrain(3, 5, 6, 1, 2, 4);
	//public JoshMotorControllor climber= new JoshMotorControllor(7, lerpSpeed,false);
	public DoubleSolenoid driveShifter = new DoubleSolenoid(0,1);
	public Joystick xbox360Controller;
	public Joystick xboxController;
	public Ultrasonic ultrasonic = new Ultrasonic(5, 6);
	public Encoder encoder;
	public AHRS navx = new AHRS(SPI.Port.kMXP);
	PIDController turnController = new PIDController(5.00, 1.0, 0.00020, 0, navx, this);
	float speedo = 0.2f; //TODO speed for PID DriveStraight

	public void robotInit()
	{
		encoder = new Encoder(2, 3);
		encoder.reset();
		turnController.setInputRange(-180.0f, 180.0f);
		turnController.setOutputRange(-1.0f, 1.0f);
		turnController.setAbsoluteTolerance(2.0);
		turnController.setContinuous(true);
		turnController.disable();
	}

	public void autonomousInit() {
		driveShifter.set(DoubleSolenoid.Value.kReverse);
	}

	public void autonomousPeriodic() {
		
		LogInfo("AUTO");
		driveShifter.set(DoubleSolenoid.Value.kReverse);
		drivetrain.SetLeftSpeed(.2f);
		drivetrain.SetRightSpeed(.2f);
		UpdateMotors();
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
		System.out.println("Encoder: " + encoder.get());

		//System.out.println("Ultrasonic: " + ultrasonic.getRangeInches());
		//sSystem.out.println("Navx: " + navx.getYaw());

		UpdateMotors();
		ControllerDrive();
		
	}
	
	public void testInit() {
		xbox360Controller = new Joystick(0);
		xboxController = new Joystick(1);
		float lerpSpeed = 0.5f;
		drivetrain.SetLeftSpeed(lerpSpeed);
		drivetrain.SetRightSpeed(lerpSpeed);
		SmartDashboard.putNumber("P: ", turnController.getP());
		SmartDashboard.putNumber("I: ", turnController.getI());
		SmartDashboard.putNumber("D: ", turnController.getD());
		SmartDashboard.putNumber("F: ", turnController.getF());
		SmartDashboard.putNumber("Speed: ", speedo);
	}
	public void testPeriodic() {
		if (xbox360Controller.getRawButton(4)) {
			DriveStraight(speedo, false);
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
		turnController.setP(SmartDashboard.getNumber("P: ", turnController.getP()));
		turnController.setI(SmartDashboard.getNumber("I: ", turnController.getI()));
		turnController.setD(SmartDashboard.getNumber("D: ", turnController.getD()));
		turnController.setF(SmartDashboard.getNumber("F: ", turnController.getF()));
		speedo = (float) SmartDashboard.getNumber("Speed: ", 0.2f);
		
		
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
	public void DriveStraight(float speed, boolean reverse) {
		float pidError = (float)turnController.get();
		drivetrain.SetLeftSpeed((speed * pidError) + speed); //0.6972
		drivetrain.SetRightSpeed(((speed) - (speed * pidError)) * -1); //-0.583

		speed = -speed;
		if(reverse){
			speed = -speed;
		}

		LogInfo("STRAIGHT YAW " + navx.getYaw());
		LogInfo("P: " + turnController.getP());
		LogInfo("I: " + turnController.getI());
		LogInfo("D: " + turnController.getD());
		LogInfo("F: " + turnController.getF());
	}

	public void ClearRotation() {
		navx.zeroYaw();
		turnController.setSetpoint(0);
	}

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
	
	@Override
	public void pidWrite(double output) {
		//  Auto-generated method stub

	}
}
