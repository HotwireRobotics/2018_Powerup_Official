package org.usfirst.frc.team2990.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;

public class AutoStep {
	public enum StepType {
		RotateLeft, RotateRight, Forward, UltrasonicTarget, AlignUltrasonicLeft, LeftTurnSide, NavxReset, Push, RightTurnSide, WallTrackLeft, WallTrackRight, ShootInSwitch, End
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
	public Ultrasonic LeftSideUltrasonic;
	public Ultrasonic ultradown;
	public float ultrasonicTarget;
	public Timer navxTime;
	public Timer pushtime;
	public float timecap;
	public Robot robot;
	public AutoStep(DriveTrain choochoo, AHRS gyro, Ultrasonic lsonar, Ultrasonic rsonar, Ultrasonic  leftsideultrasonic, Robot robot) {
		drivetrain = choochoo;
		navx = gyro;
		lultrasonic = lsonar;
		rultrasonic = rsonar;
		this.robot = robot;
		ultradown = leftsideultrasonic;

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
	public void RightTurnSide(float deg, float sped){
		type = StepType.RightTurnSide;
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
		speed = sped;
		timecap = time;
	}
	public void WallTrackRight(float sped){
		type = StepType.WallTrackRight;
		this.speed = sped;
	}
	public void WallTrackLeft(float sped){
		this.speed = sped;
		type = StepType.WallTrackLeft;
	}
	public void ShootInSwitch(){

		type = StepType.ShootInSwitch;
	}

	public void Update() {
		LogInfo("NavX: " + navx.getYaw());
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
			if (lultrasonic.getRangeInches() > ultrasonicTarget) {
				drivetrain.DriveStraight(speed, false);
				robot.ArmGoHigh();
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
			if((Math.abs(navx.getYaw()) < rotateTarget)){
				drivetrain.SetLeftSpeed(speed);
				robot.ArmGoHigh();
			}else{
				isDone= true;
			}
		}
		if(type == StepType.RightTurnSide){
			if((Math.abs(navx.getYaw()) < rotateTarget)){
				drivetrain.SetRightSpeed(-speed);
				robot.ArmGoHigh();
			}else{
				isDone= true;
			}
		}
		if(type == StepType.NavxReset){
			if(navxTime.get() > timecap){
				navxTime.stop();
				robot.pancake.set(DoubleSolenoid.Value.kForward);
				isDone = true;
			} else {
				robot.ArmDoScale();
			}
		}
		if(type == StepType.Push){
			robot.pancake.set(DoubleSolenoid.Value.kReverse);
			robot.ArmDoSwitch();
			System.out.println("Pushtime: " + pushtime.get());
			System.out.println("TimeCap:" + timecap);
			if(pushtime.get() < timecap){
				drivetrain.SetRightSpeed(-speed);
				drivetrain.SetLeftSpeed(speed);
				if(pushtime.get() >= .3f){
					robot.outtake();
					robot.ArmDoSwitch();
				}
			}else{
				isDone = true;
			}
		}
		if(type == StepType.WallTrackLeft){
			System.out.println("In here");
			System.out.println("Ultradown:" + ultradown.getRangeInches());
			if(ultradown.getRangeInches() > 3.2 ){
				drivetrain.SetRightSpeed(speed);
				drivetrain.SetLeftSpeed(-speed);

			}else{
				drivetrain.SetBothSpeed(0);
				isDone = true;
			}
		}
		if(type == StepType.ShootInSwitch){
			robot.outtake();
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