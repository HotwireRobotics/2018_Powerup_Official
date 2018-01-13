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
	public DriveTrain drivetrain = new DriveTrain(3, 5, 6, 1, 2, 4);
	//public JoshMotorControllor climber= new JoshMotorControllor(7, lerpSpeed,false);
	public DoubleSolenoid driveShifter = new DoubleSolenoid(0,1);
	public Joystick xbox360Controller;
	public Joystick xboxController;


	public void autonomousInit() {
		driveShifter.set(DoubleSolenoid.Value.kReverse);
	}

	public void autonomousPeriodic() {
		driveShifter.set(DoubleSolenoid.Value.kReverse);
		drivetrain.SetLeftSpeed(.2f);
		drivetrain.SetRightSpeed(.2f);
		UpdateMotors();
	}

	public void teleopInit() {


		xbox360Controller = new Joystick(0);
		xboxController = new Joystick(1);


		float lerpSpeed = 0.5f;
		drivetrain.SetLeftSpeed(lerpSpeed);
		drivetrain.SetRightSpeed(lerpSpeed);
	}
	public void teleopPeriodic() {
		
		UpdateMotors();
		{
			float horJoystick = TranslateController((float)xbox360Controller.getRawAxis(0));
			float verJoystick = TranslateController((float)xbox360Controller.getRawAxis(5));

			drivetrain.SetRightSpeed(verJoystick + horJoystick);
			drivetrain.SetLeftSpeed(-verJoystick + horJoystick);
			System.out.println("horizontal" + horJoystick);
			System.out.println("vertical" + verJoystick);
		}
	}

	public void testPeriodic() {
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
}

