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
import edu.wpi.first.wpilibj.Compressor;

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
 
	
	public float delay;
	// Sensors
	// {
	public AHRS navx = new AHRS(SPI.Port.kMXP);
	public Ultrasonic frontUltrasonic = new Ultrasonic(0,1);
	//public Ultrasonic rightsideultrasonic = new Ultrasonic(2, 3);
	//public Ultrasonic leftsideultrasonic = new Ultrasonic(4, 5);
	public Ultrasonic intakeUltrasonic = new Ultrasonic(2,3);
	//public Ultrasonic rightIultrasonic = new Ultrasonic(8, 9);
	public CameraServer camera;
	public Potentiometer pot = new AnalogPotentiometer(0);
	// }

	// Drivetrain
	// {
	public DriveTrain drivetrain = new DriveTrain(10, 9, 7, 8, navx); 
	// }

	// neumatics
	// {
	public DoubleSolenoid flapper = new DoubleSolenoid(0,1);
	public DoubleSolenoid pancake = new DoubleSolenoid(5,4);
	
	// }

	// Joysticks
	// {
	public Joystick xbox360Controller;
	public Joystick operator;
	public Joystick debug;
	// }

	// Shooting
	// {
	public Victor wheelOne = new Victor(3); //l
	public Victor wheelTwo = new Victor(2); //r
	//public Victor wheelThree = new Victor(7); //na
	//public Victor wheelFour = new Victor(6); //na
	public Double ultrasonicFinal;
	float speedo = 0.2f;
	public boolean On;
	public boolean Off;
	public boolean intakeMoving;
	public boolean ultraTrigger;
	public double intakeUltraValue;
	public Timer Timer = new Timer();
	public int delayShoot = 0;
	public boolean ultraToggle;
	// arm
	public float forwardArmOneSpeed = .4f;
	public float backwardArmOneSpeed = -.2f;
	public Talon armOne = new Talon(0);
	public Talon armTwo = new Talon(1);
	public float potTarget;
	public boolean armMove;
	
	public enum ArmTarget
	{
		None, Hold, Switch, Scale;
	};
	public ArmTarget armTarget;

	public double SwitchP = 5f;
	public double SwitchI= .5f;
	public double SwitchD = 0f;
	public float SwitchF = 0f;
	public float SwitchTarget = 0.65f;

	public double ScaleP = 0.0f;
	public double ScaleI= 0.0f;
	public double ScaleD = 0.0f;
	public float ScaleF = 5f;
	public float ScaleTarget = 0.56f;
	public float HighTarget = .47f;

	public float AutoP = 6.0f;
	public float AutoI= 0.5f;
	public float AutoD = 0f;
	public float AutoF = 0f;	

	public float HoldP = 1.0f;
	public float HoldI = 0.0f;
	public float HoldD = 0.0f;
	public float HoldF = 0.0f;
	public float HoldTarget = 0.62f;
	public Timer pancakeTimer = new Timer();
	public boolean doPidArmControl = true;
	public boolean pancakeOut;

	// }

	// Auto
	// {
	public AutoStep step = new AutoStep(drivetrain, navx, frontUltrasonic, this);
	public AutoStep[] Switch = new AutoStep[6];
	public AutoStep[] Scale = new AutoStep[1];
	public AutoStep[] Cross = new AutoStep[2];
	public AutoStep[] AutonomousUsing;
	public int currentStep = 0;
	// }
	public float pitch;
	public boolean pitchvalue;
	public boolean ultradown;
	public float largestZ;
	PIDController armController;
	public double potVal;

	public boolean cubeHold;
	
	//public SendableChooser autoSelection= new SendableChooser();
	public void robotInit() {
		
		//autoSelection.addObject("Switch", AutonomousUsing = Switch);
		//autoSelection.addObject("Cross", AutonomousUsing = Cross);
		
		
		//SmartDashboard.putData("Autonomous Side Selection", autoSelection);
		
		SmartDashboard.putNumber("Auto Delay", delay);
		
		pitch = navx.getPitch() * 1000;

		armController = new PIDController(6.5f, 0.3, 7.0, 0, pot, this);
		//armController.setInputRange(3.0f, 10.0f);
		//armController.setOutputRange(-0.5f, 0.5f);
		armController.setAbsoluteTolerance(1000);
		armController.setContinuous(false);
		armController.disable();

		// encoder = new Encoder(3,4);
		// encoder.reset();

		frontUltrasonic.setAutomaticMode(true);
		intakeUltrasonic.setAutomaticMode(true);
		//leftsideultrasonic.setAutomaticMode(true);

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
		delay = (float) SmartDashboard.getNumber("Auto Delay", delay);

		
		drivetrain.SetBreak();
	
		//Line Cross auto
		Cross[0] = new AutoStep(drivetrain, navx, frontUltrasonic, this);
		Cross[1] = new AutoStep(drivetrain, navx, frontUltrasonic, this);
		
		Cross[0].NavxReset(delay);
		Cross[1].TimedForward(0.8f, 7);
		

		// switch auto
		Switch[0] = new AutoStep(drivetrain, navx, frontUltrasonic,  this);
		Switch[1] = new AutoStep(drivetrain, navx, frontUltrasonic,  this);
		Switch[2] = new AutoStep(drivetrain, navx, frontUltrasonic,  this);
		Switch[3] = new AutoStep(drivetrain, navx, frontUltrasonic,  this);
		Switch[4] = new AutoStep(drivetrain, navx, frontUltrasonic,  this);
		Switch[5] = new AutoStep(drivetrain, navx, frontUltrasonic,  this);
		
		Switch[0].NavxReset(0.20f);
		Switch[0].InitStep();
		
		if (DriverStation.getInstance().getGameSpecificMessage().charAt(0) == 'L') {
			Switch[1].RightTurnSide(12f, 1f);
			System.out.println("Left");
		} else {
			Switch[1].LeftTurnSide(7f, 1f);
			System.out.println("Right");
		}
		Switch[2].UltrasonicTarget(28f, 0.5f);
		Switch[3].Push(1f, .4f);
		//Switch[4].Backup(1f, 0.6f, 17f);
		//Switch[5].Straighten(0.6f);
		
		// scale auto
		Scale[0] = new AutoStep(drivetrain, navx, frontUltrasonic, this);
		Scale[0].WallTrackLeft(0.3f);
		
		AutonomousUsing = Switch;
		
		currentStep = 0;
	}

	public void autonomousPeriodic() {

		LogInfo("Switch[" + currentStep + "]");

		if (currentStep < AutonomousUsing.length) {
			AutonomousUsing[currentStep].Update();
			if (AutonomousUsing[currentStep].isDone) {
				currentStep++;
				if (currentStep < AutonomousUsing.length) {
					AutonomousUsing[currentStep].InitStep();
				}
			}
		}

		if( AutonomousUsing == Switch && currentStep > 6){
			wheelOne.set(0);
			wheelTwo.set(0);
		}

		UpdateMotors();
	}

	public void teleopInit() {
		//DriveTrain.Speed = 0;
		armMove = false;
		armController.disable();
		debug = new Joystick(3);

		drivetrain.SetCoast();

		xbox360Controller = new Joystick(0);
		operator = new Joystick(1);
		
		intakeUltrasonic.setAutomaticMode(true);
		frontUltrasonic.setAutomaticMode(true);

		float lerpSpeed = 0.5f;
		drivetrain.SetLeftSpeed(lerpSpeed);
		drivetrain.SetRightSpeed(lerpSpeed);

		SmartDashboard.putNumber("Hold P: ", HoldP);
		SmartDashboard.putNumber("Hold I: ", HoldI);
		SmartDashboard.putNumber("Hold D: ", HoldD);

		armTarget = ArmTarget.None;

		SmartDashboard.putNumber("Scale P", ScaleP);
		SmartDashboard.putNumber("Scale I", ScaleI);
		SmartDashboard.putNumber("Scale D", ScaleD); 
		SmartDashboard.putNumber("Scale F", ScaleF);
		
		SmartDashboard.putNumber("Switch P", SwitchP);
		SmartDashboard.putNumber("Switch I", SwitchI);
		SmartDashboard.putNumber("Switch D", SwitchD);
	}
	
	public void teleopPeriodic() {	
		
		if(pancake.get() == DoubleSolenoid.Value.kForward){
			pancakeOut = true;
		}else{
			pancakeOut = false;
		}
		
				
		SmartDashboard.putNumber("Pot value: ", pot.get());

		if (navx.getRawGyroZ() > largestZ) {
			largestZ = navx.getRawGyroZ();
		}

		if (xbox360Controller.getRawButton(8)) {
			largestZ = 0;
		}
		ControllerDrive();

		SmartDashboard.putNumber("Front Ultrasonic: ", frontUltrasonic.getRangeInches());
	//	SmartDashboard.putNumber("Left Side Ultrasonic: ", leftsideultrasonic.getRangeInches());
		SmartDashboard.putNumber("Yaw: ", navx.getYaw());
		SmartDashboard.putBoolean("Ultrasonic Down", ultradown);
		SmartDashboard.putNumber("Pot: ", pot.get());
				
				
				
		SmartDashboard.putBoolean("Pancake", pancakeOut);
		LogInfo("Pancake: " + pancake.get());
		SmartDashboard.putNumber("Intake Ultrasonic: ", intakeUltrasonic.getRangeInches());
		
		//TODO


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

		double smoothing = 0.1f;
		intakeUltraValue = (smoothing * intakeUltrasonic.getRangeInches()) + ((1 - smoothing) * intakeUltraValue);
		SmartDashboard.putNumber("INTAKE ULTRA ", intakeUltraValue);
		
		if(operator.getRawButton(5)){
			armTarget = ArmTarget.Switch;
			pancake.set(DoubleSolenoid.Value.kForward);
			ultraTrigger = false;
		}else if(operator.getRawButton(6)){
			armTarget = ArmTarget.Scale;
			pancake.set(DoubleSolenoid.Value.kForward);
			ultraTrigger = false;
		}else{
			pancake.set(DoubleSolenoid.Value.kReverse);
			armTarget = ArmTarget.None;
			if (ultraTrigger && !operator.getRawButton(1)) {
				if (intakeUltraValue < 4.5f) { //4.5
					cubeHold = true;
				}
				
				if (cubeHold)
				{
					pancake.set(DoubleSolenoid.Value.kForward);
				}
			} else {
				cubeHold = false;
				pancake.set(DoubleSolenoid.Value.kReverse);
			}
		}

		// operator controls

		if (delayShoot == 0) {
			Timer.start();
		}

		if (operator.getRawButton(1)) {
			intake();
			intakeMoving = true;
			flapper.set(DoubleSolenoid.Value.kForward);
			ultraTrigger = true;
		} else if (operator.getRawButton(4)) {
			ultraTrigger = false;
			outtake();
			pancake.set(DoubleSolenoid.Value.kReverse);
			intakeMoving = true;
		} else if (operator.getPOV() > 180 && operator.getPOV() < 359) {
			// switch shooting 

			flapper.set(DoubleSolenoid.Value.kReverse);
			intakeMoving = true;
			ultraTrigger = false;

			shoot(1);

		} else if (operator.getPOV() > 5 && operator.getPOV() < 175) {
			// scale shooting

			if (Timer.get() >= 1) {
				flapper.set(DoubleSolenoid.Value.kReverse);
				pancake.set(DoubleSolenoid.Value.kReverse);
				ultraTrigger = false;
			} else {
				ultraTrigger = false;
				flapper.set(DoubleSolenoid.Value.kForward);
				delayShoot++;
				pancakeTimer.reset();
			}
			if (Timer.get() >= .25) {
				shoot(1);
				intakeMoving = true;
			}

		} else if (operator.getRawButton(2)) {
			intake();
			intakeMoving = true;
		}else if(operator.getRawButtonPressed(8)){
			shoot(1);
			intakeMoving = true;
			ultraTrigger = false;
		}else if(operator.getRawButton(3)){
			flapper.set(DoubleSolenoid.Value.kForward);
		}else{
			intakeMoving = false;
			flapper.set(DoubleSolenoid.Value.kReverse);
			wheelOne.set(0);
			wheelTwo.set(0);
			delayShoot = 0;
			Timer.stop();
			Timer.reset();
		}



		if (intakeMoving == false) {
			wheelOne.set(0);
			wheelTwo.set(0);
		}


		// arm targets
		{
			HoldTarget = (float) SmartDashboard.getNumber("Hold Targer", HoldTarget);
			HoldP = (float) SmartDashboard.getNumber("Hold P: ", HoldP);
			HoldI = (float) SmartDashboard.getNumber("Hold I: ", HoldI);
			HoldD = (float) SmartDashboard.getNumber("Hold D: ", HoldD);


			if (armTarget == ArmTarget.Hold) {
				LogInfo("Holding");

				armController.setP(HoldP);
				armController.setI(HoldI);
				armController.setD(HoldD);

				armController.setSetpoint(HoldTarget);
				armController.enable();
				doPidArmControl = true;

			} else if (armTarget == ArmTarget.Switch) {
				ArmDoSwitch();
			} else if (armTarget == ArmTarget.Scale) {
				ArmDoScale();
			}else if(armTarget == ArmTarget.None){
				armController.disable();
				armController.reset();
				doPidArmControl = true;

			} else {		
				armController.disable();
				armController.reset();
				doPidArmControl = true;
			}
		}

		ScaleP = SmartDashboard.getNumber("Scale P", ScaleP);
		ScaleI = SmartDashboard.getNumber("Scale I", ScaleI);
		ScaleD = SmartDashboard.getNumber("Scale D", ScaleD);
		
		
		
		SwitchP = SmartDashboard.getNumber("Switch P", SwitchP);
		SwitchI = SmartDashboard.getNumber("Switch I", SwitchI);
		SwitchD = SmartDashboard.getNumber("Switch D", SwitchD);
		//TODO
		
		
		
		UpdateMotors();
	}

	public void testInit() {
		debug = new Joystick(3);
		xbox360Controller = new Joystick(0);
		operator = new Joystick(1);
		float lerpSpeed = 0.5f;
		drivetrain.SetLeftSpeed(lerpSpeed);
		drivetrain.SetRightSpeed(lerpSpeed);
		flapper.set(DoubleSolenoid.Value.kOff);

		SmartDashboard.putNumber("Pot Target", potTarget);
		//SmartDashboard.putNumber("Switch P: ", SwitchP);
		//SmartDashboard.putNumber("Switch I: ", SwitchI);
		//SmartDashboard.putNumber("Switch D: ", SwitchD);
		//SmartDashboard.putNumber("Switch F: ", SwitchF);
		//SmartDashboard.putNumber("Scale P: ", ScaleP);
		//SmartDashboard.putNumber("Scale I: ", ScaleI);
		//SmartDashboard.putNumber("Scale D: ", ScaleD);
		//SmartDashboard.putNumber("Scale F: ", ScaleF);

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
			doPidArmControl = false;
		}

		if (debug.getRawButton(1)) {
			armController.reset();
		}
	}

	public void ArmDoScale() {
		armController.setP(ScaleP);
		armController.setI(ScaleI);
		armController.setD(ScaleD);
		armController.setF(ScaleF);
			
		if (pot.get() > 0.6f) {
			doPidArmControl = false;
			armOne.set(0.75);
			armTwo.set(0.75);
			armController.disable();
			SmartDashboard.putString("Scale power: ", "70% Power");
			//TODO
		} else {
			doPidArmControl = false;
			armOne.set(0.35);
			armTwo.set(0.35);
			armController.disable();
			SmartDashboard.putString("Scale power: ", "35% Power");
		}

	}
	
	public void ArmDoSwitch() {
		
		armController.setP(SwitchP);
		armController.setI(SwitchI);
		armController.setD(SwitchD);

		armController.setSetpoint(SwitchTarget);
		armController.enable();
		doPidArmControl = true;

	}
	public void ArmDoSwitchAuto() {
		armController.setP(AutoP);
		armController.setI(AutoI);
		armController.setD(AutoD);

		armController.setSetpoint(SwitchTarget -.05);
		armController.enable();
		doPidArmControl = true;

	}
	public void ArmGoHigh() {
		armOne.set(0.4);
		armOne.set(0.4);

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
		wheelOne.set(-wheelspeed);
		wheelTwo.set(wheelspeed);
		//wheelThree.set(wheelspeed);
		//wheelFour.set(wheelspeed);



	}

	public void outtake() {
		float wheelspeed = 0.70f;
		wheelOne.set(wheelspeed);
		wheelTwo.set(-wheelspeed);
		//wheelThree.set(-wheelspeed);
		//wheelFour.set(-wheelspeed);


	}

	public void shoot(float speed) {

		float wheelspeed = speed;
		wheelOne.set(wheelspeed);
		wheelTwo.set(-wheelspeed);
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
		}
	}
}