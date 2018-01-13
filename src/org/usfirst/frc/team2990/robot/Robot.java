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
<<<<<<< HEAD
	public JoshMotorControllor leftMotorTop= new JoshMotorControllor(1, lerpSpeed, false);
	public JoshMotorControllor leftMotorBottom1 = new JoshMotorControllor(2, lerpSpeed, false);
	public JoshMotorControllor leftMotorBottom2 = new JoshMotorControllor(4, lerpSpeed, false);
	public JoshMotorControllor rightMotorTop= new JoshMotorControllor(3, lerpSpeed, false);
	public JoshMotorControllor rightMotorBottom1 = new JoshMotorControllor(5, lerpSpeed, false);
	public JoshMotorControllor rightMotorBottom2 = new JoshMotorControllor(6, lerpSpeed, false);
	//public JoshMotorControllor climber = new JoshMotorControllor(7, lerpSpeed,false);
=======
	public JoshMotorControllor leftMotorTop= new JoshMotorControllor(12, lerpSpeed, false);
	public JoshMotorControllor leftMotorBottom = new JoshMotorControllor(9, lerpSpeed, false);
	public JoshMotorControllor rightMotorTop= new JoshMotorControllor(15, lerpSpeed, false);
	public JoshMotorControllor rightMotorBottom = new JoshMotorControllor(7, lerpSpeed,false);;
	public JoshMotorControllor climber = new JoshMotorControllor(7, lerpSpeed,false);
>>>>>>> 559dd8732e0e391812ac3c64e0b48729a27e93ee
	
	public Joystick xbox360Controller;
	public Joystick xboxController;
	
	public void autonomousInit() {

	}
	
	public void autonomousPeriodic() {
		SetRightMotors(.2f);
	}

	public void teleopInit() {
		
		
		xbox360Controller = new Joystick(0);
		xboxController = new Joystick(1);
		
		
		float lerpSpeed = 0.5f;
		leftMotorTop.accelValue = lerpSpeed;
		leftMotorBottom1.accelValue = lerpSpeed;
		leftMotorBottom2.accelValue = lerpSpeed;
		rightMotorTop.accelValue = lerpSpeed;
		rightMotorBottom1.accelValue = lerpSpeed;	
		rightMotorBottom2.accelValue = lerpSpeed;	
	}
	public void teleopPeriodic() {
		UpdateMotors();
		{
			float horJoystick = 0;
			float verJoystick = 0;

			float epsilon = 0.2f;
			float leftInput = TranslateController((float)xbox360Controller.getRawAxis(0));
			if (leftInput > epsilon || leftInput < -epsilon) {
				horJoystick = leftInput;
			}
			float rightInput = TranslateController((float)xbox360Controller.getRawAxis(5));
			if (rightInput > epsilon || rightInput < -epsilon) {
				verJoystick = rightInput;
			}

			SetLeftMotors(verJoystick + horJoystick);
			SetRightMotors(-verJoystick + horJoystick);
		}
	}

	public void testPeriodic() {
	}
	public void SetLeftMotors(float speed){
		leftMotorTop.target = -speed;
		leftMotorBottom1.target = speed;
		leftMotorBottom2.target = speed;
	}

	public void SetRightMotors(float speed) {
		rightMotorTop.target = -speed;
		rightMotorBottom1.target = speed;
		rightMotorBottom2.target = speed;
	}

	public void UpdateMotors() {
		leftMotorTop.UpdateMotor();
		leftMotorBottom1.UpdateMotor();
		leftMotorBottom2.UpdateMotor();
		rightMotorTop.UpdateMotor();
		rightMotorBottom1.UpdateMotor();
		rightMotorBottom2.UpdateMotor();

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

