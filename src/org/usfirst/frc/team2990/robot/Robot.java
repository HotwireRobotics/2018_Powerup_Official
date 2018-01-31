package org.usfirst.frc.team2990.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.CameraServer;

import javax.sound.midi.ControllerEventListener;

import org.usfirst.frc.team2990.robot.DriveTrain;
import edu.wpi.cscore.UsbCamera;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Ultrasonic;


public class Robot extends IterativeRobot {

	//Sensors
	//{
	public AHRS navx = new AHRS(SPI.Port.kMXP);
	public Ultrasonic lultrasonic = new Ultrasonic(8,9);
	public Ultrasonic rultrasonic = new Ultrasonic(2,3);
	public Ultrasonic rightsideultrasonic = new Ultrasonic(4,5);
	public Ultrasonic leftsideultrasonic = new Ultrasonic(6,7);
	public CameraServer camera;
	//}

	//Drivetrain
	//{
	public DriveTrain drivetrain = new DriveTrain(8, 7, 10, 9, navx);
	//}

	//neumatics
	//{
	public DoubleSolenoid flapper1 = new DoubleSolenoid(2,3);
	public DoubleSolenoid flapper2 = new DoubleSolenoid(4,5);
	//}

	//Joysticks
	//{
	public Joystick xbox360Controller;
	public Joystick operator;
	//}

	//Shooting
	//{
	public WPI_TalonSRX wheelOne = new WPI_TalonSRX(7);
	public WPI_TalonSRX wheelTwo = new WPI_TalonSRX(8);
	public WPI_TalonSRX wheelThree = new WPI_TalonSRX(9);
	public WPI_TalonSRX wheelFour = new WPI_TalonSRX(10);

	//}

	//Shooting
	//{
	public Double ultrasonicFinal;		
	float speedo = 0.2f; 
	public boolean On; 
	public boolean Off;
	//}

	//Auto
	//{
	public AutoStep step = new AutoStep(drivetrain, navx, lultrasonic, rultrasonic, leftsideultrasonic, this);
	public AutoStep[] Switch = new AutoStep[5];
	public AutoStep[] Scale = new AutoStep[1];
	public AutoStep[] AutonomousUsing;
	public int currentStep = 0;
	//}
	public float pitch;
	public boolean pitchvalue;
	public boolean ultradown;
	public float largestZ;
	
	public void robotInit()
	{
		pitch = navx.getPitch() *1000;
		//PIDController turnController;
		//turnController = new PIDController(5.00, 1.0, 0.00020, 0, ultrasonicFinal, this);
		//turnController.setInputRange(-180.0f, 180.0f);
		//turnController.setOutputRange(-1.0f, 1.0f);
		//turnController.setAbsoluteTolerance(2.0);
		//turnController.setContinuous(true);
		//turnController.disable();


		//encoder = new Encoder(3,4);
		//encoder.reset();

		lultrasonic.setAutomaticMode(true);
		rultrasonic.setAutomaticMode(true);
		rightsideultrasonic.setAutomaticMode(true);
		leftsideultrasonic.setAutomaticMode(true);
		

		//Camera
		{
			camera = CameraServer.getInstance();
			UsbCamera usbCam = camera.startAutomaticCapture(); 
			usbCam.setResolution(250,  210);
			usbCam.setFPS(30);
		}
		largestZ = 0;
	}

	public void autonomousInit() {
		
		String gameColors = DriverStation.getInstance().getGameSpecificMessage();
		if (gameColors.charAt(0) == 'L')
		{
			// do left switch 
		} else {
			// do right switch
		}
		
		Switch[0] = new AutoStep(drivetrain, navx, lultrasonic, rultrasonic, leftsideultrasonic, this);
		Switch[1] = new AutoStep(drivetrain, navx, lultrasonic, rultrasonic, leftsideultrasonic, this);
		Switch[2] = new AutoStep(drivetrain, navx, lultrasonic, rultrasonic,leftsideultrasonic, this);
		Switch[3] = new AutoStep(drivetrain, navx, lultrasonic, rultrasonic, leftsideultrasonic, this);
		Switch[4] = new AutoStep(drivetrain, navx, lultrasonic, rultrasonic,leftsideultrasonic, this);
		Switch[0].NavxReset(.05f);
		Scale[0] = new AutoStep(drivetrain, navx, lultrasonic, rultrasonic,leftsideultrasonic, this);
		if (gameColors.charAt(0) == 'L')
		{
			Switch[1].LeftTurnSide(15f, -.5f);
		} else
		{
			Switch[1].RightTurnSide(15f, -.5f);
		}

		Switch[2].UltrasonicTarget(30f, -.7f);
		Switch[3].AlignUltrasonicLeft(10f, -.4f);
		Switch[4].Push(5f, .3f);
		currentStep = 0;
		Switch[0].InitStep();
		Scale[0].WallTrackLeft(0.3f);
		AutonomousUsing = Scale;
	}

	public void autonomousPeriodic() {
		LogInfo("Left Ultrasonic: "+ lultrasonic.getRangeInches());
		LogInfo("Right Ultrasonic: "+ rultrasonic.getRangeInches());
		LogInfo("AUTO");
		//step.Update();
		UpdateMotors();
		LogInfo("Switch[" + currentStep + "]");

		if (currentStep < AutonomousUsing.length){
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
		lultrasonic.setAutomaticMode(true);
		rultrasonic.setAutomaticMode(true);
		LogInfo("TELEOP");


		xbox360Controller = new Joystick(0);
		operator = new Joystick(1);


		float lerpSpeed = 0.5f;
		drivetrain.SetLeftSpeed(lerpSpeed);
		drivetrain.SetRightSpeed(lerpSpeed);
	}

	public void teleopPeriodic() {
		//LogInfo("Ultrasonic: " + ultrasonic.getRangeInches());
		//LogInfo("Navx: " + navx.getYaw());

		System.out.println("NavxZ:" + navx.getRawGyroZ());
		if(navx.getRawGyroZ() > largestZ){
			largestZ = navx.getRawGyroZ();
		}
		
		if(leftsideultrasonic.getRangeInches() < 3.2){
			ultradown = true;
		}else{
			ultradown = false;
		}
		if(xbox360Controller.getRawButton(8)){
			largestZ = 0;
		}
		UpdateMotors();
		ControllerDrive();
		SmartDashboard.putNumber("Left Ultrasonic: ", lultrasonic.getRangeInches());
		SmartDashboard.putNumber("Right Ultrasonic: ", rultrasonic.getRangeInches());
		SmartDashboard.putNumber("Right Side Ultrasonic: ", rightsideultrasonic.getRangeInches());
		SmartDashboard.putNumber("Left Side Ultrasonic: ", leftsideultrasonic.getRangeInches());
		SmartDashboard.putNumber("Yaw:", navx.getYaw());
		SmartDashboard.putBoolean("Ultrasonic Down", ultradown);
		SmartDashboard.putNumber("Navx Z: ", navx.getRawGyroZ());
		SmartDashboard.putNumber("Max Navx Z: ", largestZ);

		
		if(pitch - navx.getPitch() < 0){
			pitchvalue = false;
		}else{
			pitchvalue = true;
		}
		System.out.println("Left Ultrasonic: "+ lultrasonic.getRangeInches());
		System.out.println("Right Ultrasonic: "+ rultrasonic.getRangeInches());
		System.out.println("Right Side Ultrasonic: "+ rightsideultrasonic.getRangeInches());
		System.out.println("Left side Ultrasonic: "+ leftsideultrasonic.getRangeInches());
		
		if(operator.getRawButton(7)){
			navx.reset();
		}
		/*
		//operator controls
		{
			boolean intakemoving= false;
			if(operator.getRawButton(2)){
				//flapper1.set(DoubleSolenoid.Value.kForward);
				//flapper2.set(DoubleSolenoid.Value.kForward);
				//intake();
				//intakemoving= true;
				flapper1.set(DoubleSolenoid.Value.kForward);
				flapper2.set(DoubleSolenoid.Value.kForward);
				isOn();
			}else{
				flapper1.set(DoubleSolenoid.Value.kReverse);
				flapper2.set(DoubleSolenoid.Value.kReverse);
			}if(operator.getRawButton(3)){
				intake();
				intakemoving= true;
			}if(operator.getRawButton(1)){
				outtake();
				intakemoving= true;
			}if(!intakemoving){
				wheelOne.set(0);
				wheelTwo.set(0);	
				wheelThree.set(0);
				wheelFour.set(0);
			}
			if(operator.getRawButton(8)){
				shoot();
			}
*/
			
			
			/*
			if (On=true){
				if (lultrasonic.getRangeInches()==0 && rultrasonic.getRangeInches()==0){
					isOff();
				}else{
					intake();
				}
			}
			if (Off=true){
				wheelOne.set(0);
				wheelTwo.set(0);	

			}
			 */
		//}
	}

	public void testInit() {
		xbox360Controller = new Joystick(0);
		operator = new Joystick(1);
		float lerpSpeed = 0.5f;
		drivetrain.SetLeftSpeed(lerpSpeed);
		drivetrain.SetRightSpeed(lerpSpeed);
		flapper1.set(DoubleSolenoid.Value.kOff);
		/*
		SmartDashboard.putNumber("P: ", turnController.getP());
		SmartDashboard.putNumber("I: ", turnController.getI());
		SmartDashboard.putNumber("D: ", turnController.getD());
		SmartDashboard.
("F: ", turnController.getF());
		 */
		SmartDashboard.putNumber("Speed: ", speedo);
	}
	public void testPeriodic() {

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
		/*turnController.setP(SmartDashboard.getNumber("P: ", turnController.getP()));
		turnController.setI(SmartDashboard.getNumber("I: ", turnController.getI()));
		turnController.setD(SmartDashboard.getNumber("D: ", turnController.getD()));
		turnController.setF(SmartDashboard.getNumber("F: ", turnController.getF()));*/
		speedo = (float) SmartDashboard.getNumber("Speed: ", 0.2f);
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


	//custom classes
	public void LogInfo(String info) {
		System.out.println(info + ";    ");
	}

	public void ControllerDrive()
	{
		float horJoystick = TranslateController((float)xbox360Controller.getRawAxis(0));
		float verJoystick = TranslateController((float)xbox360Controller.getRawAxis(5));

		drivetrain.SetRightSpeed(verJoystick + horJoystick);
		drivetrain.SetLeftSpeed(-verJoystick + horJoystick);

	}

	public void intake(){
		float wheelspeed = .8f;
		wheelOne.set(-wheelspeed);
		wheelTwo.set(-wheelspeed);
		wheelThree.set(wheelspeed);
		wheelFour.set(wheelspeed);
	}
	public void outtake(){
		float wheelspeed = .99f;
		wheelOne.set(wheelspeed);
		wheelTwo.set(wheelspeed);
		wheelThree.set(-wheelspeed);
		wheelFour.set(-wheelspeed);

	}
	public void shoot(){

		float wheelspeed = 1f;
		wheelOne.set(wheelspeed);
		wheelTwo.set(wheelspeed);
		wheelThree.set(-wheelspeed);
		wheelFour.set(-wheelspeed);


	}
	public void isOn(){
		On = true;
		Off = false;

	}
	public void isOff(){
		On = false;
		Off = true;
	}

}
