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
	public Potentiometer pot = new AnalogPotentiometer(0, 360, 30);
	// }

	// Drivetrain
	// {
	public DriveTrain drivetrain = new DriveTrain(10, 9, 7, 8, navx);
	// }

	// neumatics
	// {
	public DoubleSolenoid flapper1 = new DoubleSolenoid(0, 1);
	public DoubleSolenoid flapper2 = new DoubleSolenoid(2, 3);
	public DoubleSolenoid piston = new DoubleSolenoid(4,5);
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
	// arm
	public float forwardArmOneSpeed = .4f;
	public float backwardArmOneSpeed = -.2f;
	public Talon armOne = new Talon(4);
	public Talon armTwo = new Talon(5);
	public float potTarget;
	

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
	
	
	public void robotInit() {
		
		pitch = navx.getPitch() * 1000;

		armController = new PIDController(5.00, 1.0, 0.00020, 0, pot, this);
		armController.setInputRange(-180.0f, 180.0f);
		armController.setOutputRange(-1.0f, 1.0f);
		armController.setAbsoluteTolerance(2.0);
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
		
		debug = new Joystick(3);
		
		armOne.setSpeed(0);
		armTwo.setSpeed(0);
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
		SmartDashboard.putNumber("Arm P:", armController.getP());
		SmartDashboard.putNumber("Arm I:", armController.getI());
		SmartDashboard.putNumber("Arm D:", armController.getD());
		SmartDashboard.putNumber("Arm F:", armController.getF());
		// LogInfo("Ultrasonic: " + ultrasonic.getRangeInches());
		// LogInfo("Navx: " + navx.getYaw());
		// leftsideultrasonic).getP());
		// leftsideultrasonic).getI());
		// leftsideultrasonic).getD());
		SmartDashboard.putNumber("Pot: ", pot.get());
		if (xbox360Controller.getRawButton(7)) {
			armOne.setSpeed(.6);
			armTwo.setSpeed(.6);
		}

		// System.out.println("NavxZ:" + navx.getRawGyroZ());
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
		SmartDashboard.putNumber("Yaw:", navx.getYaw());
		SmartDashboard.putBoolean("Ultrasonic Down", ultradown);
		SmartDashboard.putNumber("Right Intake Ultrasonic: ", rightIultrasonic.getRangeInches());
		SmartDashboard.putNumber("Left Intake Ultrasonic: ", leftIultrasonic.getRangeInches());

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
			armOne.set(SmartDashboard.getNumber("Forward Arm Speed ONE: ", forwardArmOneSpeed));
			armTwo.set(SmartDashboard.getNumber("Forward Arm Speed TWO: ", forwardArmOneSpeed));
		} else if (xbox360Controller.getRawButton(7)) {
			armOne.set(SmartDashboard.getNumber("Backward Arm Speed ONE: ", backwardArmOneSpeed));
			armTwo.set(SmartDashboard.getNumber("Backward Arm SpeedTWO: ", backwardArmOneSpeed));
		} else {
			armOne.set(0);
			armTwo.set(0);
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

		
		if (operator.getRawButton(1)) {
			intake();
			piston.set(DoubleSolenoid.Value.kForward);
			intakeMoving = true;
			flapper1.set(DoubleSolenoid.Value.kForward);
			flapper2.set(DoubleSolenoid.Value.kForward);
			ultraTrigger = true;
		} else if (operator.getRawButton(4)) {
			ultraTrigger = false;
			outtake();
			intakeMoving = true;
		} else if (operator.getRawAxis(3) > .1) {
			flapper1.set(DoubleSolenoid.Value.kReverse);
			flapper2.set(DoubleSolenoid.Value.kReverse);
			intakeMoving = true;
			ultraTrigger = false;
			shoot(1);
			armChange(130);
			// Scale
		} else if (operator.getRawAxis(2) > .1) {
			ultraTrigger = false;
			flapper1.set(DoubleSolenoid.Value.kReverse);
			flapper2.set(DoubleSolenoid.Value.kReverse);
			armChange(379);
			// switch
		} else if (operator.getRawButton(2)) {
			intake();
			intakeMoving = true;
		}else if(operator.getRawButtonPressed(8)){
			shoot(1);
			intakeMoving = true;
			ultraTrigger = false;
		}else if(operator.getRawButton(5)){
			flapper1.set(DoubleSolenoid.Value.kReverse);
			flapper2.set(DoubleSolenoid.Value.kReverse);
		} else {
			LogInfo("ARM STATIC");
			intakeMoving = false;
			flapper1.set(DoubleSolenoid.Value.kReverse);
			flapper2.set(DoubleSolenoid.Value.kReverse);
			wheelOne.set(0);
			wheelTwo.set(0);
			piston.set(DoubleSolenoid.Value.kForward);
			//wheelThree.set(0);
			//wheelFour.set(0);
		}

		if (intakeMoving = false) {
			wheelOne.set(0);
			wheelTwo.set(0);
			piston.set(DoubleSolenoid.Value.kForward);
		//	wheelThree.set(0);
			//wheelFour.set(0);
		}
		if (ultraTrigger == true) {
			if (rightIultrasonic.getRangeInches() >= 5f || leftIultrasonic.getRangeInches() >= 5f) {
				//
			} else {
				wheelOne.set(0);
				wheelTwo.set(0);
				piston.set(DoubleSolenoid.Value.kForward);
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

	}

	public void testPeriodic() {

		SmartDashboard.getNumber("Pot Target", potTarget);
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
		
		if (operator.getRawButton(1)) {
			outtake();
		} else {
			wheelOne.set(0);
			wheelTwo.set(0);
		}


		if (operator.getRawButton(2)) {
			flapper1.set(DoubleSolenoid.Value.kForward);
			flapper2.set(DoubleSolenoid.Value.kForward);
		} else {
			flapper1.set(DoubleSolenoid.Value.kReverse);
			flapper2.set(DoubleSolenoid.Value.kReverse);
		}
		armController.setSetpoint(250);
		if(debug.getRawButton(8)){
		armController.enable();
		}else{
			armController.disable();
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
		//System.out.println(info + ";    ");
	}

	public void ControllerDrive() {
		float horJoystick = TranslateController((float) xbox360Controller.getRawAxis(0));
		float verJoystick = TranslateController((float) xbox360Controller.getRawAxis(5));

		drivetrain.SetRightSpeed(verJoystick + horJoystick);
		drivetrain.SetLeftSpeed(-verJoystick + horJoystick);

	}

	public void intake() {
		float wheelspeed = .4f;
		wheelOne.set(wheelspeed);
		wheelTwo.set(-wheelspeed);
		//wheelThree.set(wheelspeed);
		//wheelFour.set(wheelspeed);

	}

	public void outtake() {
		float wheelspeed = 1f;
		wheelOne.set(-wheelspeed);
		wheelTwo.set(wheelspeed);
		piston.set(DoubleSolenoid.Value.kReverse);
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

	public void armChange(int targetValue) {
		if (pot.get() < targetValue) {
			intakeMoving = true;
			//LogInfo("SHOOT");
			shoot(1);
			armOne.setSpeed(.35);
			armTwo.setSpeed(.35);
		} else if (pot.get() >= targetValue) {
			//LogInfo("GOING UP");
			armOne.setSpeed(.6);
			armTwo.setSpeed(.6);
		} else {
			//LogInfo("OUT OF RANGE");
			intakeMoving = false;
			armOne.setSpeed(0);
			armTwo.setSpeed(0);
		}
	}

	@Override
	public void pidWrite(double output) {
		armOne.set(output);
		armTwo.set(output);
		LogInfo("PID output:" + output);
		
	}
}
