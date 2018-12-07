package org.usfirst.frc.team2990.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDInterface;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import javax.sound.midi.ControllerEventListener;

import org.usfirst.frc.team2990.robot.DriveTrain;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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
	//public Ultrasonic intakeUltrasonic = new Ultrasonic(2,3);
	public DigitalInput leftSwitch = new DigitalInput(2);
	public DigitalInput rightSwitch = new DigitalInput(3);
	public CameraServer camera;
	public Potentiometer pot = new AnalogPotentiometer(0);
	public DigitalInput limitSwitch = new DigitalInput(4);
	// }

	// Drivetrain
	// {
	public DriveTrain drivetrain = new DriveTrain(10, 9, 7, 8, navx); 
	// }

	// neumatics
	// {
	//public DoufbleSolenoid ramps = new DoubleSolenoid(7,6); //R1 :2,3 R2: 7,6
	public DoubleSolenoid flapper = new DoubleSolenoid(2,3); //R1: 7,6 R2: 2,3
	public DoubleSolenoid pancake = new DoubleSolenoid(4,5);

	// }

	// Joysticks
	// {
	public Joystick xbox360Controller;
	public Joystick operator;
	public Joystick debug;
	// }

	// Shooting
	// {
	public TalonSRX wheelOne = new TalonSRX(1); //l
	public TalonSRX wheelTwo = new TalonSRX(4); //r
	//public Victor wheelThree = new Victor(7); //na
	//public Victor wheelFour = new Victor(6); //na
	public Double ultrasonicFinal;
	float speedo = 0.2f;
	public boolean On;
	public boolean Off;
	public boolean intakeMoving;
	public boolean ultraTrigger;
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

	public double SwitchP = 5f; //5
	public double SwitchI= .6f;
	public double SwitchD = 6.0f;
	public float SwitchF = 0f;
	public float SwitchTarget = 0.62f; //.62  //0.89 //TODO

	public double ScaleP = 0.0f;
	public double ScaleI= 0.0f;
	public double ScaleD = 0.0f;
	public float ScaleF = 5f; //5 //TODO
	public float ScaleTarget = 0.56f; //.56  //0.82 //TODO
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
	public AutoStep[] Switch = new AutoStep[7];
	public AutoStep[] Shoot = new AutoStep[1];
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
	public String gameMessage;

	public String crossLine = "Normal Switch";
	public boolean cubeHold;

	public void robotInit() {

		SmartDashboard.putString("autoChoice", "N");

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
		//intakeUltrasonic.setAutomaticMode(true);
		//leftsideultrasonic.setAutomaticMode(true);

		// Camera
		{
			camera = CameraServer.getInstance();

			UsbCamera usbCam = camera.startAutomaticCapture();
			usbCam.setResolution(250, 210);
			usbCam.setFPS(30);

			MjpegServer s = CameraServer.getInstance().addServer("DashStream");
			s.setSource(usbCam);
		}
		largestZ = 0;
	}

	public void disabledPeriodic() {
		delay = (float) SmartDashboard.getNumber("Auto Delay", delay);
		crossLine = SmartDashboard.getString("Cross Line; Normal Switch; Shoot Switch", crossLine);

		SmartDashboard.putString("Cross Line; Normal Switch; Shoot Switch", crossLine);
		SmartDashboard.putNumber("Auto Delay", delay);

		xbox360Controller = new Joystick(0);
		operator = new Joystick(1);
	}

	public void autonomousInit() {

		delay = (float) SmartDashboard.getNumber("Auto Delay", delay);
		crossLine = SmartDashboard.getString("Cross Line; Normal Switch; Shoot Switch", crossLine);

		System.out.println("Auto Delay " + delay);
		System.out.println("Auto Choice " + crossLine);



		drivetrain.SetBreak();

		//Cross Line  auto
		Cross[0] = new AutoStep(drivetrain, navx, frontUltrasonic, this);
		Cross[1] = new AutoStep(drivetrain, navx, frontUltrasonic, this);


		Cross[0].NavxReset(delay + 0.2f);
		Cross[0].InitStep();
		Cross[1].TimedForward(0.8f, 1.75f);


		//Normal Switch  auto
		Switch = new AutoStep[13];
		for (int i = 0; i < Switch.length; i++) {
			Switch[i] =  new AutoStep(drivetrain, navx, frontUltrasonic,  this);
		}


		Switch[0].NavxReset(0.20f + delay);
		Switch[0].InitStep();
		Switch[1].RobotTurn(1.0f, 14f, 8f, 0.0f, 0.0f); //r,l
		Switch[2].Wait(0.1f);
		Switch[3].UltrasonicTarget(22f, 0.7f); //28
		Switch[4].Push(2.0f, .3f);
		Switch[5].Backup(0.4f, 1.5f);
		Switch[6].Straighten(-0.4f, 41.0f, -21.0f); //l, r

		// These are for second cube auto
		//Switch[7].ForwardPickup(0.4f, 1.4f, 2.0f);
		//Switch[8].Backup(0.4f, 1.30f);
		//Switch[9].StraightenInvert(0.6f, 6.0f, 0.0f); //l,r
		//Switch[10].ArmGoHigh();
		//Switch[11].UltrasonicTarget(22f, 0.8f);
		//Switch[12].Push(5.0f, .3f);
		// -----------


		//Switch[6].Wait(0.2f);
		//Switch[7].Straighten(0.4f, 30);
		//Switch[5].Straighten(0.6f);

		//Shoot Switch  auto
		Shoot = new AutoStep[5];
		for (int i = 0; i < Shoot.length; i++) {
			Shoot[i] =  new AutoStep(drivetrain, navx, frontUltrasonic,  this);
		}

		Shoot[0].NavxReset(0.0f + delay);
		Shoot[0].InitStep();
		Shoot[1].RobotTurn(1.0f, 13f, 8f, 1.0f, 1.0f);
		Shoot[2].ShootSwitch(1.0f, 1.0f, 1.0f);
		//TODO find the correct degrees for right side rotate
		Shoot[3].Rotate(0.0f, 10.0f, 0.25f, -1);
		//TODO test step 4
		Shoot[4].ForwardPickup(0.25f, 1.5f, 0.25f);

		if(crossLine == "Cross Line"){
			System.out.println("Cross");
			AutonomousUsing = Cross;
		}else if (crossLine == "Normal Switch"){
			AutonomousUsing = Switch;
		}else if (crossLine == "Shoot Switch"){
			AutonomousUsing = Shoot;
		}else{
			System.out.println("ERROR: Auto select misspelled; Defaulting to Shoot Switch");
			AutonomousUsing = Switch;
		} //TODO fix auto select

		currentStep = 0;
	}

	public void autonomousPeriodic() {

		LogInfo("Switch[" + currentStep + "] ---------------- ");

		if (currentStep < AutonomousUsing.length) {
			AutonomousUsing[currentStep].Update();
			if (AutonomousUsing[currentStep].isDone) {
				currentStep++;
				if (currentStep < AutonomousUsing.length) {
					AutonomousUsing[currentStep].InitStep();
				}
			}
		}



		UpdateMotors();
	}

	public void teleopInit() {



		//DriveTrain.Speed = 0;
		armMove = false;
		armController.disable();
		debug = new Joystick(3);
		//SmartDashboard.putNumber("Intake Ultrasonic", intakeUltrasonic.getRangeInches());

		drivetrain.SetCoast();

		xbox360Controller = new Joystick(0);
		operator = new Joystick(1);

		//intakeUltrasonic.setAutomaticMode(true);
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

		flapper.set(DoubleSolenoid.Value.kReverse);
	}

	public void teleopPeriodic() {

		NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry tableX = table.getEntry("tx");
		NetworkTableEntry tableY = table.getEntry("ty");
		NetworkTableEntry tableA = table.getEntry("ta");
		NetworkTableEntry tableL = table.getEntry("tl");
		NetworkTableEntry tableS = table.getEntry("ts");
		NetworkTableEntry tableV = table.getEntry("tv");
		double tx = tableX.getDouble(0);
		double ty = tableY.getDouble(0);
		double ta = tableA.getDouble(0);
		double tl = tableL.getDouble(0);
		double ts = tableS.getDouble(0);
		double tv = tableV.getDouble(0);
		float txTarget = 1f;
		float txSpeed = 0.5f;
		float taTarget = 1.5f;
		float taSpeed = 0.5f;

		//System.out.println("Xvalue  " + x);
		//System.out.println("Yvalue  " + y);
		System.out.println("Avalue  " + ta);

		//SmartDashboard.putString("this!", 1234);

		if (xbox360Controller.getRawButton(1)) {
			NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
		} else {
			NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
		}

		if (xbox360Controller.getRawButton(2)) {

			if (tv == 0){
				drivetrain.SetRightSpeed(1f);
				drivetrain.SetLeftSpeed(1f);	
			} else if(tx>1.25){
				drivetrain.SetRightSpeed((float) (tx*tx));
				drivetrain.SetLeftSpeed((float) (tx*tx));
			} else if(tx<-1.25){
				drivetrain.SetRightSpeed((float) (-1*(tx*tx)));
				drivetrain.SetLeftSpeed((float) (-1*(tx*tx)));
			}else if (tx>=-1.25 && tx<=1.25){
				drivetrain.SetRightSpeed((float) ((taTarget-ta)*taSpeed));
				drivetrain.SetLeftSpeed((float) (-1*((taTarget-ta)*taSpeed)));
			}else{
				drivetrain.SetRightSpeed(0);
				drivetrain.SetLeftSpeed(0);
			}

		} else {
			drivetrain.SetRightSpeed(0);
			drivetrain.SetLeftSpeed(0);
		}
		//SmartDashboard.putNumber("Intake Ultrasonic", intakeUltrasonic.getRangeInches());
		//SmartDashboard.getNumber("Intake Ultrasonic", intakeUltrasonic.getRangeInches());
		//if(xbox360Controller.getRawButton(1)){
		//ramps.set(DoubleSolenoid.Value.kReverse);
		//}else{
		//ramps.set(DoubleSolenoid.Value.kForward);
		//}

		//LogInfo("" + armTarget);

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
		//ControllerDrive();

		SmartDashboard.putNumber("Front Ultrasonic: ", frontUltrasonic.getRangeInches());
		//	SmartDashboard.putNumber("Left Side Ultrasonic: ", leftsideultrasonic.getRangeInches());
		SmartDashboard.putNumber("Yaw: ", navx.getYaw());
		SmartDashboard.putBoolean("Ultrasonic Down", ultradown);
		SmartDashboard.putNumber("Pot: ", pot.get());

		SmartDashboard.putBoolean("Pancake", pancakeOut);	


		if (xbox360Controller.getRawButton(6)) {
			armOne.set(0.8f);
			armTwo.set(0.8f);
		} else if (xbox360Controller.getRawButton(7)) {
			armOne.set(SmartDashboard.getNumber("Backward Arm Speed ONE: ", backwardArmOneSpeed));
			armTwo.set(SmartDashboard.getNumber("Backward Arm SpeedTWO: ", backwardArmOneSpeed));
			armMove = false;
		} else {
			armMove = false;
		}


		if(operator.getRawButton(5)){
			armTarget = ArmTarget.Switch;
			if (operator.getPOV() < 0 && !operator.getRawButton(4)) {
				pancake.set(DoubleSolenoid.Value.kForward);
			}
			ultraTrigger = false;
		} else if(operator.getRawButton(6)){
			armTarget = ArmTarget.Scale;
			if (operator.getPOV() < 0) {
				pancake.set(DoubleSolenoid.Value.kForward);
			}
			ultraTrigger = false;
		} else {
			armTarget = ArmTarget.None;
			if (ultraTrigger && !operator.getRawButton(1)) {
				//if (intakeUltrasonic.getRangeInches() < 3.5f){
				if(leftSwitch.get() && rightSwitch.get()) {
					cubeHold = true;
				}

				if (cubeHold) {
					pancake.set(DoubleSolenoid.Value.kForward);
					intakeMoving = false;
				} else {
					intakeMoving = true;
					intake();
				}
			} else {
				intakeMoving = false;
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
			if(armTarget == armTarget.Switch){
				pancake.set(DoubleSolenoid.Value.kReverse);
			}
			intakeMoving = true;

		} else if ((operator.getPOV() > 270 || operator.getPOV() < 90) && operator.getPOV() != -1) {
			// scale shooting
			LogInfo("High Scale Shot");

			if (Timer.get() >= 1.4f) {
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

		}else if(operator.getPOV() > 90 && operator.getPOV() < 270){
			//low scale shooting
			LogInfo("High Scale Shot");

			if (Timer.get() >= 1.4f) {
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
				shoot(0.8f);
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
			if (operator.getPOV() < 0) {
				flapper.set(DoubleSolenoid.Value.kReverse); 
				delayShoot = 0;
				Timer.stop();
				Timer.reset();
			}
		}

		//LogInfo("POV: " + operator.getPOV());
		//LogInfo("CAKE - " + pancake.get());
		if (!intakeMoving) {
			wheelOne.set(ControlMode.PercentOutput, 0);
			wheelTwo.set(ControlMode.PercentOutput, 0);
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

				if(pot.get() < .69f){ //.69
					armOne.set(0);
					armTwo.set(0);	
				}

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
		SmartDashboard.putNumber("Yaw", navx.getYaw());

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

		SmartDashboard.getNumber("Yaw", navx.getYaw());

		if (debug.getRawButton(5)) {
			//intake();
		} else {
			//wheelOne.set(0);
			//wheelTwo.set(0);
		}


		if (debug.getRawButton(2)) {
			pancake.set(DoubleSolenoid.Value.kForward);
			//flapper.set(DoubleSolenoid.Value.kForward);
		} else {
			pancake.set(DoubleSolenoid.Value.kReverse);
			//flapper.set(DoubleSolenoid.Value.kReverse);
		}

		if (debug.getRawButton(4)) {
			//pancake.set(DoubleSolenoid.Value.kForward);
			flapper.set(DoubleSolenoid.Value.kForward);
		} else {
			//pancake.set(DoubleSolenoid.Value.kReverse);
			flapper.set(DoubleSolenoid.Value.kReverse);
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

		if (pot.get() > 0.61f) { //.64
			doPidArmControl = false;
			armOne.set(0.90);
			armTwo.set(0.90);
			armController.disable();
			SmartDashboard.putString("Scale power: ", "70% Power");
			//TODO
		} else {
			doPidArmControl = false;
			armOne.set(0.40);
			armTwo.set(0.40);
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
		armOne.set(0.7);
		armOne.set(0.7);

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
		float wheelspeed = 0.5f;

		wheelOne.set(ControlMode.PercentOutput, wheelspeed);
		wheelTwo.set(ControlMode.PercentOutput, -wheelspeed);
		//wheelThree.set(wheelspeed);
		//wheelFour.set(wheelspeed);



	}

	public void outtake() {
		float wheelspeed = 0.60f;
		wheelOne.set(ControlMode.PercentOutput, -wheelspeed);
		wheelTwo.set(ControlMode.PercentOutput, wheelspeed);
		//wheelThree.set(-wheelspeed);
		//wheelFour.set(-wheelspeed);


	}

	public void lowScale(float speed){
		float wheelspeed = speed;
		wheelOne.set(ControlMode.PercentOutput, wheelspeed);
		wheelTwo.set(ControlMode.PercentOutput, -wheelspeed, ScaleD);
		//wheelThree.set(-wheelspeed);
		//wheelFour.set(-wheelspeed);


	}
	public void shoot(float speed) {

		float wheelspeed = speed;
		wheelOne.set(ControlMode.PercentOutput, -wheelspeed);
		wheelTwo.set(ControlMode.PercentOutput, wheelspeed);
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