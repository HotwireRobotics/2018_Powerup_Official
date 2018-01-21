package org.usfirst.frc.team2990.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;

public class AutoStep {
	public enum StepType {
		RotateLeft, RotateRight, Forward, UltrasonicTarget, AlignUltrasonicLeft, LeftTurnSide, NavxReset, Push
	}

	public StepType type;
	public DriveTrain drivetrain;
	public AHRS navx;
	public float rotateTarget;
	public float speed;
	public boolean isDone;
	public Encoder encoder;
	public float encoderTarget;
	public Ultrasonic lultrasonic;
	public Ultrasonic rultrasonic;
	public float ultrasonicTarget;
	public Timer navxTime;
	public Timer pushtime;
	public float timecap;
	public Robot robot;
	public AutoStep(DriveTrain choochoo, AHRS gyro, Encoder encode, Ultrasonic lsonar, Ultrasonic rsonar, Robot robot) {
		drivetrain = choochoo;
		navx = gyro;
		encoder = encode;
		lultrasonic = lsonar;
		rultrasonic = rsonar;
		this.robot = robot;

		navxTime = new Timer();
		pushtime = new Timer();
		isDone = false;
	}

	public void MoveForward(float encode, float sped) {
		type = StepType.Forward;
		encoderTarget = encode;
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
	public void UltrasonicTarget(float dist, float sped) {
		type = StepType.UltrasonicTarget;
		lultrasonic.setAutomaticMode(true);
		rultrasonic.setAutomaticMode(true);
		ultrasonicTarget = dist;
		speed = sped;
	}
	public void LeftTurnSide(float deg, float sped){
		type = StepType.LeftTurnSide;
		rotateTarget = deg;
		speed = sped;		
	}
	public void NavxReset(float time){
		type = StepType.NavxReset;
		timecap = time;
	}
	public void AlignUltrasonicLeft(float dist, float sped) {
		type = StepType.AlignUltrasonicLeft;
		ultrasonicTarget = dist;
		speed = sped;
	}
	public void Push(float time, float sped){
		type = StepType.Push; 
		timecap = time;
	}

	public void Update() {
		LogInfo("NavX: " + navx.getYaw());
		LogInfo("Encoder: " + encoder.getDistance());
		drivetrain.SetLeftSpeed(0.0f);
		drivetrain.SetRightSpeed(0.0f);
		double adjustment = Math.pow(35.0f, speed);
		LogInfo("Adj: " +adjustment);
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
		if (type == StepType.UltrasonicTarget) {
			if (lultrasonic.getRangeInches() > ultrasonicTarget && rultrasonic.getRangeInches() > ultrasonicTarget) {
				drivetrain.DriveStraight(speed, false);
			} else {
				isDone = true;
			}
			if (type == StepType.AlignUltrasonicLeft) {
				if (lultrasonic.getRangeInches() > ultrasonicTarget) {
					drivetrain.SetRightSpeed(speed);
				} else {
					isDone = true;
				}
			}
		}
		if(type == StepType.LeftTurnSide){
			if((Math.abs(navx.getYaw()) < rotateTarget - adjustment)){
				drivetrain.SetLeftSpeed(speed);
			}else{
				isDone= true;
			}
		}
		if(type == StepType.NavxReset){
			if(navxTime.get() > timecap){
				navxTime.stop();
				isDone = true;
			}
		}
		if(type == StepType.Push){
			if(pushtime.get() < timecap){
				drivetrain.SetBothSpeed(speed);
			}else{
				
				robot.shoot();
			}
		}
	}

	public void InitStep()
	{
		navx.reset();
		navxTime.start();
		pushtime.start();
	}

	public void LogInfo(String info) {
		System.out.println(info + ";    ");
	}

}