package org.usfirst.frc.team2990.robot;

import org.usfirst.frc.team2990.robot.Robot.ArmTarget;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;

public class AutoStep {
	public enum StepType {
		RotateLeft, RotateRight, Forward, UltrasonicTarget, AlignUltrasonicLeft, LeftTurnSide, NavxReset, Push, RightTurnSide, WallTrackLeft, WallTrackRight, ShootInSwitch, Backup, Straighten, TimedForward, RobotTurn, Wait, ForwardPickup
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
	public Ultrasonic LeftSideUltrasonic;
	public Ultrasonic ultradown;
	public float ultrasonicTarget;
	public Timer navxTime;
	public Timer pushtime;
	public Timer backtime;
	public Timer forwardTime;
	public Timer timer;
	public Timer rotateTime;
	public Timer pickupTime;
	public Timer grabTime;
	public Timer takeTime;
	public float timecap;
	public float timecap2;
	public Robot robot;
	private float rotateTargetLeft;
	private float rotateTargetRight;
	public double firstUltra;
	public AutoStep(DriveTrain choochoo, AHRS gyro, Ultrasonic lsonar, Robot robot) {
		drivetrain = choochoo;
		navx = gyro;
		lultrasonic = lsonar;
		this.robot = robot;

		navxTime = new Timer();
		pushtime = new Timer();
		backtime = new Timer();
		timer = new Timer();
		pickupTime = new Timer();
		grabTime= new Timer();
		rotateTime = new Timer();
		takeTime = new Timer();
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
	public void Backup(float sped, float time){
		this.speed = sped;
		timecap = time;
		type = StepType.Backup;
	}
	public void Straighten(float sped, float time, float time2){
		this.speed = sped;
		timecap = time;
		timecap2 = time2;
		type = StepType.Straighten;
	}

	public void TimedForward(float sped, float time){
		this.speed = sped;
		timecap = time;
		type = StepType.TimedForward;

	}
	public void RobotTurn(float sped, float degLeft, float degRight){
		this.speed = sped;
		rotateTargetLeft = degLeft;
		rotateTargetRight = degRight;
		type = StepType.RobotTurn;

	}
	public void Wait(float time){
		timecap = time;
		type = StepType.Wait;
	}
	public void ForwardPickup(float sped, float time, float time2){
		timecap = time;
		timecap2 = time2;
		this.speed = sped;
		type = StepType.ForwardPickup;
	}

	public void Update() {

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
		if (type == StepType.UltrasonicTarget) {
			if (lultrasonic.getRangeInches() > ultrasonicTarget) {
				drivetrain.DriveStraight(speed, false);
				robot.ArmDoSwitchAuto();
				LogInfo("Ultrasonic: " + robot.frontUltrasonic.getRangeInches());
			} else {
				isDone = true;
			}
		}
		if (type == StepType.AlignUltrasonicLeft) {
			if (lultrasonic.getRangeInches() > ultrasonicTarget) {
				drivetrain.SetRightSpeed(speed);
			} else {
				isDone = true;
			}
		}
		if (type == StepType.Backup){
			robot.pancake.set(DoubleSolenoid.Value.kForward);
			robot.wheelOne.set(ControlMode.PercentOutput, 0.0f);
			robot.wheelTwo.set(ControlMode.PercentOutput, 0.0f);
			if (backtime.get() < timecap) {
				drivetrain.DriveStraight(-speed, false);
				robot.wheelOne.set(ControlMode.PercentOutput, 0.0);
				robot.wheelTwo.set(ControlMode.PercentOutput, 0.0);
			} else {
				isDone = true;
			}
		}
		if(type == StepType.Straighten){
			if(DriverStation.getInstance().getGameSpecificMessage().charAt(0) == 'L'){
				if( rotateTime.get() < timecap){
					drivetrain.SetLeftSpeed(-speed);
					drivetrain.SetRightSpeed(-speed);
				}else{
					robot.armController.reset();
					isDone = true;
				}
			}else{
				if( rotateTime.get() < timecap2){
					drivetrain.SetLeftSpeed(speed);
					drivetrain.SetRightSpeed(speed);
				}else{
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
				robot.armController.reset();
			}
		}
		if(type == StepType.RightTurnSide){
			if((Math.abs(navx.getYaw()) < rotateTarget)){
				drivetrain.SetRightSpeed(-speed);
				robot.ArmGoHigh();
			}else{
				robot.armController.reset();
				isDone= true;
			}
		}
		if(type == StepType.NavxReset){
			System.out.println("HEY " + navxTime.get());
			if(navxTime.get() > timecap){
				navx.reset();
				robot.pancake.set(DoubleSolenoid.Value.kForward);
				isDone = true;
			} else {
				navx.reset();
				robot.gameMessage = DriverStation.getInstance().getGameSpecificMessage();
			}
		}
		if(type == StepType.Push){
			robot.pancake.set(DoubleSolenoid.Value.kReverse);
			robot.ArmDoSwitch();
			System.out.println("Pushtime: " + pushtime.get());
			System.out.println("TimeCap:" + timecap);
			if(pushtime.get() < timecap){
				robot.pancake.set(DoubleSolenoid.Value.kReverse);
				drivetrain.SetRightSpeed(-speed);
				drivetrain.SetLeftSpeed(speed);
				if(pushtime.get() >= 0.3f){
					float wheelspeed = 0.5f;
					robot.wheelOne.set(ControlMode.PercentOutput, wheelspeed);
					robot.wheelTwo.set(ControlMode.PercentOutput, -wheelspeed);
					robot.ArmDoSwitch();
				}
			}else{
				isDone = true;
			}
		}
		if(type == StepType.ForwardPickup){
			robot.pancake.set(DoubleSolenoid.Value.kReverse);
			//robot.armTarget = ArmTarget.None;
			robot.flapper.set(DoubleSolenoid.Value.kForward);
			//float wheelspeed = 0.5f;
			//robot.wheelOne.set(-wheelspeed);
			//robot.wheelTwo.set(wheelspeed);
			robot.armController.disable();
			robot.armController.reset();
			robot.doPidArmControl = true;
			LogInfo("In here");
			if(pickupTime.get() < timecap){

				drivetrain.DriveStraight(speed, false);
				//robot.intake();
			}else if(pickupTime.get() > timecap){
				if(grabTime.get() > timecap2){
					robot.flapper.set(DoubleSolenoid.Value.kReverse);
					if(robot.rightSwitch.get() == true && robot.leftSwitch.get() == true){
						isDone = true;
					}else{
					//	robot.wheelOne.set(-wheelspeed);
					//	robot.wheelTwo.set(wheelspeed);
						if(takeTime.get() > 1.5f){
							robot.flapper.set(DoubleSolenoid.Value.kForward);
							if(takeTime.get() > 2.0f){
								robot.flapper.set(DoubleSolenoid.Value.kReverse);
							}
						}
					}

				}
			}
		}
		if(type == StepType.Wait){
			if(timer.get() > timecap){
				isDone = true;
			}else{
				//do nothing
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
		if(type == StepType.TimedForward){
			LogInfo("Forward");
			if(pushtime.get() > timecap){
				isDone = true;
			}else{
				drivetrain.DriveStraight(speed, false);
			}
		}
		if(type == StepType.RobotTurn){
			if(DriverStation.getInstance().getGameSpecificMessage().charAt(0) == 'L'){
				if((Math.abs(navx.getYaw()) < rotateTargetLeft)){
					drivetrain.SetRightSpeed(-speed);
					robot.ArmGoHigh();
				}else{
					robot.armController.reset();
					isDone= true;	
				}
			}else{
				if((Math.abs(navx.getYaw()) < rotateTargetRight)){
					drivetrain.SetLeftSpeed(speed);
					robot.ArmGoHigh();
				}else{
					isDone= true;
					robot.armController.reset();
				}
			}
		}
	}


	public void InitStep()
	{
		navxTime.start();
		pushtime.start();
		backtime.start();
		timer.start();
		rotateTime.start();
		pickupTime.start();
		grabTime.start();
		takeTime.start();
	}

	public void LogInfo(String info) {
		System.out.println(info + ";    ");
	}

}