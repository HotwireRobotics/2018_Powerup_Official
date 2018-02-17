package org.usfirst.frc.team2990.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDInterface;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CameraServer;

import javax.sound.midi.ControllerEventListener;

import org.usfirst.frc.team2990.robot.DriveTrain;
import edu.wpi.cscore.UsbCamera;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class Robot extends IterativeRobot implements PIDOutput {

	// Sensors
	// {
	public AHRS navx = new AHRS(SPI.Port.kMXP);
	public Ultrasonic lultrasonic = new Ultrasonic(4, 5);
	public Ultrasonic rultrasonic = new Ultrasonic(11, 12);
	public Ultrasonic rightsideultrasonic = new Ultrasonic(2, 3);
	public Ultrasonic leftsideultrasonic = new Ultrasonic(0, 1);
	public Ultrasonic leftIultrasonic = new Ultrasonic(6, 7);
	public Ultrasonic rightIultrasonic = new Ultrasonic(8, 9);
	public CameraServer camera;
	public Potentiometer pot = new AnalogPotentiometer(0, 1, 0);
	// }

	// Drivetrain
	// {
	public DriveTrain drivetrain = new DriveTrain(10, 9, 7, 8, navx);
	// }

	// neumatics
	// {
	public DoubleSolenoid flapper1 = new DoubleSolenoid(0, 1);
	public DoubleSolenoid flapper2 = new DoubleSolenoid(2, 3);
	public DoubleSolenoid cubeHolder = new DoubleSolenoid(4, 5);
	// }

	// Joysticks
	// {
	public Joystick xbox360Controller;
	public Joystick operator;
	public Joystick debug;
	// }

	// Shooting
	// {
	public Victor wheelOne = new Victor(8); //l
	public Victor wheelTwo = new Victor(7); //r
	//public Victor wheelThree = new Victor(7); //na
	//public Victor wheelFour = new Victor(6); //na
	// }

	// Shooting
	// {
	public Double ultrasonicFinal;
	float speedo = 0.2f;
	public boolean On;
	public boolean Off;
	public boolean intakeMoving;
	public boolean ultraTrigger;
	public Timer Timer = new Timer();
	public int delayShoot = 0;
	// arm
	public float forwardArmOneSpeed = .4f;
	public float backwardArmOneSpeed = -.2f;
	public Talon armOne = new Talon(4);
	public Talon armTwo = new Talon(5);
	public float potTarget;
	public boolean armMove;
	public boolean ultraToggle;

	public float SwitchP = 11f;
	public float SwitchI= .5f;
	public float SwitchD = 0f;
	public float SwitchF = 0f;
	public float SwitchTarget = 0.575f;

	public float ScaleP = 20.0f;
	public float ScaleI= 1.0f;
	public float ScaleD = 0f;
	public float ScaleF = 0f;
	public float ScaleTarget = 0.505f;

	public boolean doPidArmControl = true;


	// }

	// Auto
	// {
	public AutoStep step = new AutoStep(drivetrain, navx, lultrasonic, rultrasonic, leftsideultrasonic, this);
	public AutoStep[] Switch = new AutoStep[4];
	public AutoStep[] Scale = new AutoStep[1];
	public AutoStep[] AutonomousUsing;
	public int currentStep = 0;
	// }
	public float pitch;
	public boolean pitchvalue;
	public boolean ultradown;
	public float largestZ;
	PIDController armController;

	public double potVal;

	public void robotInit() {

		pitch = navx.getPitch() * 1000;

		armController = new PIDController(6.5f, 0.3, 7.0, 0, pot, this);
		//armController.setInputRange(3.0f, 10.0f);
		//armController.setOutputRange(-0.5f, 0.5f);
		armController.setAbsoluteTolerance(1000);
		armController.setContinuous(false);
		armController.disable();

		// encoder = new Encoder(3,4);
		// encoder.reset();

		lultrasonic.setAutomaticMode(true);
		rultrasonic.setAutomaticMode(true);
		rightsideultrasonic.setAutomaticMode(true);
		leftsideultrasonic.setAutomaticMode(true);

		// Camera
		{
			camera = CameraServer.getInstance();
			UsbCamera usbCam = camera.startAutomaticCapture();
			usbCam.setResolution(250, 210);
			usbCam.setFPS(30);
		}
		largestZ = 0;


	}

	public void autonomousInit() {
		drivetrain.SetBreak();
		String gameColors = DriverStation.getInstance().getGameSpecificMessage();
		if (gameColors.charAt(0) == 'L') {
			// do left switch
		} else {
			// do right switch
		}

		Switch[0] = new AutoStep(drivetrain, navx, lultrasonic, rultrasonic, leftsideultrasonic, this);
		Switch[1] = new AutoStep(drivetrain, navx, lultrasonic, rultrasonic, leftsideultrasonic, this);
		Switch[2] = new AutoStep(drivetrain, navx, lultrasonic, rultrasonic, leftsideultrasonic, this);
		Switch[3] = new AutoStep(drivetrain, navx, lultrasonic, rultrasonic, leftsideultrasonic, this);
		Switch[0].NavxReset(.06f);
		Scale[0] = new AutoStep(drivetrain, navx, lultrasonic, rultrasonic, leftsideultrasonic, this);
		if (gameColors.charAt(0) == 'L') {
			Switch[1].RightTurnSide(12f, 1f);
		} else {
			Switch[1].LeftTurnSide(11f, 1f);
		}

		Switch[2].UltrasonicTarget(30f, .8f);
		Switch[3].Push(1f, .4f);
		currentStep = 0;
		Switch[0].InitStep();
		Scale[0].WallTrackLeft(0.3f);
		AutonomousUsing = Switch;
	}

	public void autonomousPeriodic() {
		//LogInfo("Left Ultrasonic: " + lultrasonic.getRangeInches());
		//LogInfo("Right Ultrasonic: " + rultrasonic.getRangeInches());
		// LogInfo("AUTO");
		// step.Update();
		UpdateMotors();
		LogInfo("Switch[" + currentStep + "]");

		if ((currentStep != 3)) {
			intake();
		}

		if (currentStep < AutonomousUsing.length) {
			AutonomousUsing[currentStep].Update();
			if (AutonomousUsing[currentStep].isDone) {
				currentStep++;
				if (currentStep < AutonomousUsing.length) {
					AutonomousUsing[currentStep].InitStep();
				}
			}
		}
	}

	public void teleopInit() {
		armMove = false;
		armController.disable();
		debug = new Joystick(3);

		drivetrain.SetCoast();
		lultrasonic.setAutomaticMode(true);
		rultrasonic.setAutomaticMode(true);
		// LogInfo("TELEOP");

		xbox360Controller = new Joystick(0);
		operator = new Joystick(1);


		float lerpSpeed = 0.5f;
		drivetrain.SetLeftSpeed(lerpSpeed);
		drivetrain.SetRightSpeed(lerpSpeed);
	}

	public void teleopPeriodic() {

		SmartDashboard.putNumber("Pot value: ", pot.get());

		if (navx.getRawGyroZ() > largestZ) {
			largestZ = navx.getRawGyroZ();
		}

		if (leftsideultrasonic.getRangeInches() < 3.2) {
			ultradown = true;
		} else {
			ultradown = false;
		}
		if (xbox360Controller.getRawButton(8)) {
			largestZ = 0;
		}
		UpdateMotors();
		ControllerDrive();
		SmartDashboard.putNumber("Front Ultrasonic: ", lultrasonic.getRangeInches());
		SmartDashboard.putNumber("Right Side Ultrasonic: ", rightsideultrasonic.getRangeInches());
		SmartDashboard.putNumber("Left Side Ultrasonic: ", leftsideultrasonic.getRangeInches());
		SmartDashboard.putNumber("Yaw: ", navx.getYaw());
		SmartDashboard.putBoolean("Ultrasonic Down", ultradown);
		SmartDashboard.putNumber("Right Intake Ultrasonic: ", rightIultrasonic.getRangeInches());
		SmartDashboard.putNumber("Left Intake Ultrasonic: ", leftIultrasonic.getRangeInches());
		SmartDashboard.putNumber("Pot: ", pot.get());

		if (pitch - navx.getPitch() < 0) {
			pitchvalue = false;
		} else {
			pitchvalue = true;
		}
		// System.out.println("Left Ultrasonic: "+
		// lultrasonic.getRangeInches());
		// System.out.println("Right Ultrasonic: "+
		// rultrasonic.getRangeInches());
		// System.out.println("Right Side Ultrasonic: "+
		// rightsideultrasonic.getRangeInches());
		// System.out.println("Left side Ultrasonic: "+
		// leftsideultrasonic.getRangeInches());


		if (operator.getRawButton(7)) {
			navx.reset();
		}

		if (xbox360Controller.getRawButton(6)) {
			System.out.println("HERE");
			armOne.set(0.8f);
			armTwo.set(0.8f);
		} else if (xbox360Controller.getRawButton(7)) {
			armOne.set(SmartDashboard.getNumber("Backward Arm Speed ONE: ", backwardArmOneSpeed));
			armTwo.set(SmartDashboard.getNumber("Backward Arm SpeedTWO: ", backwardArmOneSpeed));
			armMove = false;
		} else {
			armMove = false;
		}

		if (debug.getRawButton(8)) {
			// turnController.setP(SmartDashboard.getNumber("P: ",
			// turnController.getP()));
			// turnController.setI(SmartDashboard.getNumber("I: ",
			// turnController.getI()));
			// turnController.setD(SmartDashboard.getNumber("D: ",
			// turnController.getD()));
			// turnController.setF(SmartDashboard.getNumber("F: ",
			// turnController.getF()));
			speedo = (float) SmartDashboard.getNumber("Speed: ", 0.2f);

		}

		// operator controls

		if (delayShoot == 0) {
			Timer.start();
		}
		if (operator.getRawButton(1)) {
			intake();
			intakeMoving = true;
			flapper1.set(DoubleSolenoid.Value.kForward);
			flapper2.set(DoubleSolenoid.Value.kForward);
			ultraTrigger = true;
		} else if (operator.getRawButton(4)) {
			ultraTrigger = false;
			outtake();
			intakeMoving = true;
		} else if (operator.getPOV() > 180 && operator.getPOV() < 359) {
			flapper1.set(DoubleSolenoid.Value.kReverse);
			flapper2.set(DoubleSolenoid.Value.kReverse);
			intakeMoving = true;
			ultraTrigger = false;

			shoot(1);
			// Scale
		} else if (operator.getPOV() > 5 && operator.getPOV() < 175) {
			if (Timer.get() >= .8) {
				flapper1.set(DoubleSolenoid.Value.kReverse);
				flapper2.set(DoubleSolenoid.Value.kReverse);
				ultraTrigger = false;
				
				LogInfo("firing");
			} else {
				ultraTrigger = false;
				flapper1.set(DoubleSolenoid.Value.kForward);
				flapper2.set(DoubleSolenoid.Value.kForward);
				delayShoot++;
				LogInfo("reving");
			}
			if (Timer.get() >= .25) {
				outtake();
				intakeMoving = true;
				LogInfo("outtake");
			}
			// scale
		} else if (operator.getRawButton(2)) {
			intake();
			intakeMoving = true;
		}else if(operator.getRawButtonPressed(8)){
			shoot(1);
			intakeMoving = true;
			ultraTrigger = false;
		}else if(operator.getRawButton(3)){
			flapper1.set(DoubleSolenoid.Value.kForward);
			flapper2.set(DoubleSolenoid.Value.kForward);
		}else{
			//LogInfo("ARM STATIC");
			intakeMoving = false;
			flapper1.set(DoubleSolenoid.Value.kReverse);
			flapper2.set(DoubleSolenoid.Value.kReverse);
			wheelOne.set(0);
			wheelTwo.set(0);
			delayShoot = 0;
			Timer.stop();
			Timer.reset();
			//wheelThree.set(0);
			//wheelFour.set(0);
		}

		//if(operator.getRawButton(7) && ultraToggle == true){
		//ultraToggle= false;
		//}else if(operator.getRawButton(7) && ultraToggle == false){
		//ultraToggle = true;
		//}

		if(operator.getRawButton(5)){
			//switch
			armController.setP(SwitchP);
			armController.setI(SwitchI);
			armController.setD(SwitchD);
			
			armController.setSetpoint(SwitchTarget);
			armController.enable();
			doPidArmControl = true;

		}else if(operator.getRawButton(6)){
			// scale
			armController.setP(ScaleP);
			armController.setI(ScaleI);
			armController.setD(ScaleD);
			
			ArmDoScale();

		} else {
			armController.disable();
			armController.reset();
			doPidArmControl = true;
			cubeHolder.set(DoubleSolenoid.Value.kReverse);
		}
		
		
		if (intakeMoving == false) {
			wheelOne.set(0);
			wheelTwo.set(0);
			//	wheelThree.set(0);
			//wheelFour.set(0);
		}else{
			//
		}
		if (ultraTrigger == true) {
			if (rightIultrasonic.getRangeInches() >= 3f || leftIultrasonic.getRangeInches() >= 3f) {
				if(ultraToggle == true){
					intake();
				}
			} else {
				//wheelOne.set(0.3f);
				//wheelTwo.set(0.3f);
				//wheelThree.set(0);
				//wheelFour.set(0);
			}
		} else {
			intakeMoving = false;
		}
	
	}

	public void testInit() {
		debug = new Joystick(3);
		xbox360Controller = new Joystick(0);
		operator = new Joystick(1);
		float lerpSpeed = 0.5f;
		drivetrain.SetLeftSpeed(lerpSpeed);
		drivetrain.SetRightSpeed(lerpSpeed);
		flapper1.set(DoubleSolenoid.Value.kOff);

		SmartDashboard.putNumber("Pot Target", potTarget);
		SmartDashboard.putNumber("Switch P: ", SwitchP);
		SmartDashboard.putNumber("Switch I: ", SwitchI);
		SmartDashboard.putNumber("Switch D: ", SwitchD);
		SmartDashboard.putNumber("Switch F: ", SwitchF);
		SmartDashboard.putNumber("Scale P: ", ScaleP);
		SmartDashboard.putNumber("Scale I: ", ScaleI);
		SmartDashboard.putNumber("Scale D: ", ScaleD);
		SmartDashboard.putNumber("Scale F: ", ScaleF);

	}

	public void testPeriodic() {

		double smoothing = 0.5f;
		potVal = (smoothing * pot.get()) + ((1 - smoothing) * potVal);
		SmartDashboard.putNumber("Pot value: ", potVal);

		potTarget = (float) SmartDashboard.getNumber("Pot Target", potTarget);

		SmartDashboard.putBoolean("PID On Target", armController.onTarget());

		/*
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
		SmartDashboard.putNumber("Left Ultrasonic: ", lultrasonic.getRangeInches());
		SmartDashboard.putNumber("Right Ultrasonic: ", rultrasonic.getRangeInches());
		SmartDashboard.putNumber("Pot: ", pot.get());
		 */
		//LogInfo("Pot Value" + potTarget);

		if (debug.getRawButton(5)) {
			//intake();
		} else {
			//wheelOne.set(0);
			//wheelTwo.set(0);
		}


		if (debug.getRawButton(2)) {
			//flapper1.set(DoubleSolenoid.Value.kForward);
			//flapper2.set(DoubleSolenoid.Value.kForward);
		} else {
			//flapper1.set(DoubleSolenoid.Value.kReverse);
			//flapper2.set(DoubleSolenoid.Value.kReverse);
		}

		if(debug.getRawButton(5)){
			//switch
			armController.setP(SmartDashboard.getNumber("Switch P: ", SwitchP));
			armController.setI(SmartDashboard.getNumber("Switch I: ", SwitchI));
			armController.setD(SmartDashboard.getNumber("Switch D: ", SwitchD));
			armController.setF(SmartDashboard.getNumber("Switch F: ", SwitchF));
			armController.setSetpoint(SwitchTarget);
			armController.enable();
			doPidArmControl = true;

		}else if(debug.getRawButton(6)){
			//Scale
			armController.setP(SmartDashboard.getNumber("Scale P: ", ScaleP));
			armController.setI(SmartDashboard.getNumber("Scale I: ", ScaleI));
			armController.setD(SmartDashboard.getNumber("Scale D: ", ScaleD));
			armController.setF(SmartDashboard.getNumber("Scale F: ", ScaleF));

			ArmDoScale();

		} else {
			armController.disable();
			armController.reset();
			doPidArmControl = true;
			cubeHolder.set(DoubleSolenoid.Value.kReverse);
		}

		if (debug.getRawButton(1)) {
			armController.reset();
		}
	}

	public void ArmDoScale()
	{
		
		if (pot.get() > 0.53) {
			doPidArmControl = false;
			armOne.set(0.6);
			armTwo.set(0.6);
			LogInfo("Steady Power to Scale");
			armController.disable();
		} else {
			doPidArmControl = true;
			armController.enable();
			armController.setSetpoint(ScaleTarget);
			LogInfo("Ready to Shoot");
			armMove = true;
			cubeHolder.set(DoubleSolenoid.Value.kForward);
		}
		
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

	// custom classes
	public void LogInfo(String info) {
		System.out.println(info + ";    ");
	}

	public void ControllerDrive() {
		float horJoystick = TranslateController((float) xbox360Controller.getRawAxis(0));
		float verJoystick = TranslateController((float) xbox360Controller.getRawAxis(5));

		drivetrain.SetRightSpeed(verJoystick + horJoystick);
		drivetrain.SetLeftSpeed(-verJoystick + horJoystick);

	}

	public void intake() {
		float wheelspeed = 0.6f;
		wheelOne.set(wheelspeed);
		wheelTwo.set(-wheelspeed);
		//wheelThree.set(wheelspeed);
		//wheelFour.set(wheelspeed);



	}

	public void outtake() {
		float wheelspeed = 1f;
		wheelOne.set(-wheelspeed);
		wheelTwo.set(wheelspeed);
		//wheelThree.set(-wheelspeed);
		//wheelFour.set(-wheelspeed);


	}

	public void shoot(float speed) {

		float wheelspeed = speed;
		wheelOne.set(-wheelspeed);
		wheelTwo.set(wheelspeed);
		//wheelThree.set(-wheelspeed);
		//wheelFour.set(-wheelspeed);
	}

	public void isOn() {
		On = true;
		Off = false;

	}

	public void isOff() {
		On = false;
		Off = true;
	}




	@Override
	public void pidWrite(double output) {
		if (doPidArmControl)
		{
			armOne.set(-output);
			armTwo.set(-output);
			SmartDashboard.putNumber("PID Output: ", -output);
			LogInfo("PIDWrite is running");
		}
	}
}