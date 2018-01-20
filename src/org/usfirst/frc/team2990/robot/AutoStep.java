package org.usfirst.frc.team2990.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.Encoder;

public class AutoStep {
	public enum StepType {
		RotateLeft, RotateRight, Forward, UltrasonicDriveStraight
	}

	public StepType type;
	public DriveTrain drivetrain;
	public AHRS navx;
	public float rotateTarget;
	public float speed;
	public boolean isDone;
	public Encoder encoder;
	public float encoderTarget;
	public Ultrasonic ultrasonic;

	public AutoStep(DriveTrain choochoo, AHRS gyro, Encoder encode, Ultrasonic sonar) {
		drivetrain = choochoo;
		navx = gyro;
		encoder = encode;
		ultrasonic = sonar;
		
		isDone = false;
	}
	
	ultrasonic.setAutomaticMode(true);
	
	public void MoveForward(float dist, float sped) {
		type = StepType.Forward;
		encoderTarget = dist;
		speed = sped;
	}
	public void RotateRight(float deg, float sped){
		rotateTarget = deg;
		type = StepType.RotateRight;
		speed = sped;
	}
	public void RotateLeft(float deg, float sped){
		rotateTarget = -deg;
		type = StepType.RotateLeft;
		speed = sped;
	}
	public void UltrasonicDriveStraight(float encode, float sped) {
		encoderTarget = encode;
		speed = sped;
	}

	public void Update() {
		LogInfo("NavX: " + navx.getYaw());
		LogInfo("Encoder: " + encoder.getDistance());
		drivetrain.SetLeftSpeed(0.0f);
		drivetrain.SetRightSpeed(0.0f);
		double adjustment = Math.pow(35.0f, speed);
		
		if (type == StepType.RotateRight) {
			if (navx.getYaw() < rotateTarget - adjustment) {
				drivetrain.SetLeftSpeed(speed);
				drivetrain.SetRightSpeed(speed);
			} else {
				isDone = true;
			}
		}
		if (type == StepType.RotateLeft) {
			if (navx.getYaw() > rotateTarget + adjustment) {
				drivetrain.SetLeftSpeed(-speed);
				drivetrain.SetRightSpeed(-speed);
			} else {
				isDone = true;
			}
		}
		if (type == StepType.Forward) {
			if (Math.abs(encoder.getDistance()) < encoderTarget) {
				drivetrain.DriveStraight(speed, false);
			} else {
				isDone = true;
			}
		}
		if (type == StepType.UltrasonicDriveStraight) {
			if (Math.abs(encoder.getDistance()) < encoderTarget) {
				
			} else {
				isDone = true;
			}
		}
	}
	
	public void InitStep()
	{
		navx.reset();
	}
	
	public void LogInfo(String info) {
		System.out.println(info + ";    ");
	}

}