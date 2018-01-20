package org.usfirst.frc.team2990.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;

public class DriveTrain implements PIDOutput {
	JoshMotorControllor joshmotorcontrollorLeftTop;
	JoshMotorControllor joshmotorcontrollorLeftBottomOne;
	JoshMotorControllor joshmotorcontrollorLeftBottomTwo;
	JoshMotorControllor joshmotorcontrollorRightTop;
	JoshMotorControllor joshmotorcontrollorRightBottomOne;
	JoshMotorControllor joshmotorcontrollorRightBottomTwo;
	float lerpSpeed = 0.8f;
	public AHRS navx;
	PIDController turnController;

	public DriveTrain(int pwm1, int pwm2, int pwm3, int pwm4, int pwm5, int pwm6, AHRS navx){
		joshmotorcontrollorLeftTop = new JoshMotorControllor(pwm1, lerpSpeed, false);
		joshmotorcontrollorLeftBottomOne= new JoshMotorControllor(pwm2, lerpSpeed, false);
		joshmotorcontrollorLeftBottomTwo = new JoshMotorControllor(pwm3, lerpSpeed, false);
		joshmotorcontrollorRightTop = new JoshMotorControllor(pwm4, lerpSpeed, false);
		joshmotorcontrollorRightBottomOne = new JoshMotorControllor(pwm5, lerpSpeed, false);
		joshmotorcontrollorRightBottomTwo= new JoshMotorControllor(pwm6, lerpSpeed, false);
		
		this.navx = navx;
		turnController = new PIDController(5.00, 1.0, 0.00020, 0, this.navx, this);
		turnController.setInputRange(-180.0f, 180.0f);
		turnController.setOutputRange(-1.0f, 1.0f);
		turnController.setAbsoluteTolerance(2.0);
		turnController.setContinuous(true);
		turnController.disable();
	}
	public void Update(){
		joshmotorcontrollorLeftTop.UpdateMotor();
		joshmotorcontrollorLeftBottomOne.UpdateMotor();
		joshmotorcontrollorLeftBottomTwo.UpdateMotor();
		joshmotorcontrollorRightTop.UpdateMotor();
		joshmotorcontrollorRightBottomOne.UpdateMotor();
		joshmotorcontrollorRightBottomTwo.UpdateMotor();
	}
	public void SetLeftSpeed(float Speed){
		joshmotorcontrollorLeftTop.target = -Speed;
		joshmotorcontrollorLeftBottomOne.target = Speed;
		joshmotorcontrollorLeftBottomTwo.target = Speed;
	}
	public void SetRightSpeed(float Speed){
		joshmotorcontrollorRightTop.target = -Speed;
		joshmotorcontrollorRightBottomOne.target = Speed;
		joshmotorcontrollorRightBottomTwo.target = Speed;
	}
	public void SetBothSpeed(float Speed){
		joshmotorcontrollorLeftTop.target = -Speed;
		joshmotorcontrollorLeftBottomOne.target = Speed;
		joshmotorcontrollorLeftBottomTwo.target = Speed;
		joshmotorcontrollorRightTop.target = -Speed;
		joshmotorcontrollorRightBottomOne.target = Speed;
		joshmotorcontrollorRightBottomTwo.target = Speed;
	}
	public void SetBreak(){
		joshmotorcontrollorRightTop.SetBrake();
		joshmotorcontrollorRightBottomOne.SetBrake();
		joshmotorcontrollorRightBottomTwo.SetBrake();
		joshmotorcontrollorLeftTop.SetBrake();
		joshmotorcontrollorLeftBottomOne.SetBrake();
		joshmotorcontrollorLeftBottomTwo.SetBrake();
	}
	public void SetCoast(){
		joshmotorcontrollorRightTop.SetCoast();
		joshmotorcontrollorRightBottomOne.SetCoast();
		joshmotorcontrollorRightBottomTwo.SetCoast();
		joshmotorcontrollorLeftTop.SetCoast();
		joshmotorcontrollorLeftBottomOne.SetCoast();
		joshmotorcontrollorLeftBottomTwo.SetCoast();

	}
	public void DriveStraight(float speed, boolean reverse) {
		float pidError = (float)turnController.get();
		SetLeftSpeed((speed * pidError) + speed); //0.6972
		SetRightSpeed(((speed) - (speed * pidError)) * -1); //-0.583

		speed = -speed;
		if(reverse){
			speed = -speed;
		}

		System.out.println("STRAIGHT YAW " + navx.getYaw() + ";");
		System.out.println("P: " + turnController.getP() + ";");
		System.out.println("I: " + turnController.getI() + ";");
		System.out.println("D: " + turnController.getD() + ";");
		System.out.println("F: " + turnController.getF() + ";");
	}
	public void ClearRotation() {
		navx.zeroYaw();
		turnController.setSetpoint(0);
	}
	@Override
	public void pidWrite(double output) {
	}

}