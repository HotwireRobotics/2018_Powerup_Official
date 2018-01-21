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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
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
	public Joystick operator;
	public Ultrasonic lultrasonic = new Ultrasonic(7,8);
	public Ultrasonic rultrasonic = new Ultrasonic(5,6);
	public Encoder encoder;
	public DoubleSolenoid flapper1 = new DoubleSolenoid(2,3);
	public DoubleSolenoid flapper2 = new DoubleSolenoid(4,5);
	float speedo = 0.2f; 

	public AutoStep step = new AutoStep(drivetrain, navx, encoder, lultrasonic, rultrasonic, this);
	public AutoStep[] Switch = new AutoStep[5];
	public int currentStep = 0;
	public WPI_TalonSRX wheelOne = new WPI_TalonSRX(7);
	public WPI_TalonSRX wheelTwo = new WPI_TalonSRX(8);

	public void robotInit()
	{
		encoder = new Encoder(3,4);
		encoder.reset();

		lultrasonic.setAutomaticMode(true);
		lultrasonic.setAutomaticMode(true);


	}

	public void autonomousInit() {
		driveShifter.set(DoubleSolenoid.Value.kForward);
		encoder.reset();
		Switch[0] = new AutoStep(drivetrain, navx, encoder, lultrasonic, rultrasonic, this);
		Switch[1] = new AutoStep(drivetrain, navx, encoder, lultrasonic, rultrasonic, this);
		Switch[2] = new AutoStep(drivetrain, navx, encoder, lultrasonic, rultrasonic, this);
		Switch[3] = new AutoStep(drivetrain, navx, encoder, lultrasonic, rultrasonic, this);
		Switch[4] = new AutoStep(drivetrain, navx, encoder, lultrasonic, rultrasonic, this);
		Switch[0].NavxReset(.05f);
		Switch[1].LeftTurnSide(15f, -.5f);
		Switch[2].UltrasonicTarget(30f, -.7f);
		Switch[3].AlignUltrasonicLeft(10f, -.4f);
		Switch[4].Push(5f, .3f);
		currentStep = 0;
		Switch[0].InitStep();
	}

	public void autonomousPeriodic() {
		LogInfo("Left Ultrasonic: "+ lultrasonic.getRangeInches());
		LogInfo("Right Ultrasonic: "+ rultrasonic.getRangeInches());
		LogInfo("AUTO");
		//step.Update();
		UpdateMotors();
		LogInfo("Switch[" + currentStep + "]");

		if (currentStep < Switch.length){
			Switch[currentStep].Update();
			if (Switch[currentStep].isDone) {
				currentStep++;
				if (currentStep < Switch.length) {
					Switch[currentStep].InitStep();
				}
			}
		}
	}

	public void teleopInit() {

		LogInfo("TELEOP");


		xbox360Controller = new Joystick(0);
		operator = new Joystick(1);


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

		//operator controls
		{
			boolean intakemoving= false;
			if(operator.getRawButton(2)){
				flapper1.set(DoubleSolenoid.Value.kForward);
				flapper2.set(DoubleSolenoid.Value.kForward);
				intake();
				intakemoving= true;
			}else{
				flapper1.set(DoubleSolenoid.Value.kReverse);
				flapper2.set(DoubleSolenoid.Value.kReverse);
			}if(operator.getRawButton(3)){
				intake();
				intakemoving= true;
			}if(operator.getRawButton(1)){
				outtake();
				intakemoving= true;
			}if(!intakemoving){
				wheelOne.set(0);
				wheelTwo.set(0);	
			}

		}
	}

	public void testInit() {
		xbox360Controller = new Joystick(0);
		operator = new Joystick(1);
		float lerpSpeed = 0.5f;
		drivetrain.SetLeftSpeed(lerpSpeed);
		drivetrain.SetRightSpeed(lerpSpeed);
		flapper1.set(DoubleSolenoid.Value.kOff);
		/*
		SmartDashboard.putNumber("P: ", turnController.getP());
		SmartDashboard.putNumber("I: ", turnController.getI());
		SmartDashboard.putNumber("D: ", turnController.getD());
		SmartDashboard.
("F: ", turnController.getF());
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
		SmartDashboard.putNumber("Left Ultrasonic: ", lultrasonic.getRangeInches());
		SmartDashboard.putNumber("Right Ultrasonic: ", rultrasonic.getRangeInches());
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

	public void intake(){
		float wheelspeed = 8f;
		wheelOne.set(-wheelspeed);
		wheelTwo.set(wheelspeed);
	}
	public void outtake(){
		float wheelspeed = 8f;
		wheelOne.set(wheelspeed);
		wheelTwo.set(-wheelspeed);
	}

	public void shoot(){
		//shoot
	}

}
