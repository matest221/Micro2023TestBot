package frc.robot.subsystems;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
// Mahesh 1/13/2023 - removing SpeedController as those classes are removed in 2023 API fix. 
// import edu.wpi.first.wpilibj.SpeedController;
// import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.commands.DriveJoystick;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.*; 

public class DriveBase extends SubsystemBase {

	WPI_TalonSRX slaveLeft;
	WPI_TalonSRX slaveRight;
	WPI_TalonSRX masterLeft;
	WPI_TalonSRX masterRight;
	public DifferentialDrive drive;
	public ADXRS450_Gyro gyro;
	// public PigeonIMU gyro;


	public Encoder leftEncoder;
	public Encoder rightEncoder;
	public static double PulsesPerRevolution = 360; //Same as PPR for E4T
	public static double PI = 3.1415926;
	public static double wheelDiameter = 8.25; // in inches
	public static double DistancePerRevolution = wheelDiameter*PI;
	public static double DistancePerPulse = DistancePerRevolution/PulsesPerRevolution;
	public static double wheelBase = 21.75; // in inches
	public static double arcLength;
	public static double kp_straight = 0.20;
	public static double kp_turn = 0.005;
	public static double voltsPerSecond = .1;

	
	private PIDController pid;
	private double kp = 1.;
	private double ki = 1.0;
	private double kd = 1.0;

	public DriveBase() {
		super();
		pid = new PIDController(kp, ki, kd);

		masterLeft = new WPI_TalonSRX(Constants.LEFT_BACK_MOTOR);
		masterRight = new WPI_TalonSRX(Constants.RIGHT_BACK_MOTOR);
		slaveLeft = new WPI_TalonSRX(Constants.LEFT_FRONT_MOTOR);
		slaveRight = new WPI_TalonSRX(Constants.RIGHT_FRONT_MOTOR);
		

		// leftEncoder = new Encoder(Constants.ENCODER_LEFTA,Constants.ENCODER_LEFTB,true,EncodingType.k4X); 
		// rightEncoder = new Encoder(Constants.ENCODER_RIGHTA,Constants.ENCODER_RIGHTB,false,EncodingType.k4X); 

		leftEncoder.setDistancePerPulse(DistancePerPulse);
		rightEncoder.setDistancePerPulse(DistancePerPulse);

		// Mahesh 1/13/2023 - SpeedController deprecated in favor of MotorController. 

		MotorController leftSide = new MotorControllerGroup(slaveLeft, masterLeft);
		MotorController rightSide = new MotorControllerGroup(slaveRight, masterRight); 
		drive = new DifferentialDrive(leftSide, rightSide);

		masterLeft.setNeutralMode(NeutralMode.Brake);
		masterRight.setNeutralMode(NeutralMode.Brake);
		masterLeft.configOpenloopRamp(voltsPerSecond);
		masterRight.configOpenloopRamp(voltsPerSecond);
		slaveLeft.set(ControlMode.Follower, Constants.LEFT_FRONT_MOTOR);
		slaveRight.set(ControlMode.Follower, Constants.RIGHT_FRONT_MOTOR);
		slaveLeft.setNeutralMode(NeutralMode.Brake);
		slaveRight.setNeutralMode(NeutralMode.Brake);
		
        masterLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        masterRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		masterLeft.configSetParameter(ParamEnum.eClearPositionOnLimitR, 1, 0, 0, 10);
		masterRight.configSetParameter(ParamEnum.eClearPositionOnLimitF, 1, 0, 0, 10);

		drive.setExpiration(0.1);
	}

	public void drive(Joystick j) {
		if (j.getRawAxis(Constants.RIGHTTRIGGER) > 0.5)
		{
			drive(0.4*j.getRawAxis(Constants.LEFTYAXIS), 0.4*j.getRawAxis(Constants.RIGHTYAXIS));
		}
		else {
			drive(j.getRawAxis(Constants.LEFTYAXIS), j.getRawAxis(Constants.RIGHTYAXIS));
		}
	}


	
	/*
	 * public void arcadeDrive(double moveSpeed, double rotateSpeed) {
	 * drive.arcadeDrive(moveSpeed, rotateSpeed); }
	 */

	public void reset() {
		drive(0.0, 0.0);
	}
	public void right(double speed) {
		drive(-speed, speed);
	}
	public void left(double speed) {
		drive(speed, -speed);
	}



	

	public void drive(double leftSpeed, double rightSpeed) {
		drive.tankDrive(-leftSpeed, rightSpeed, false);
	}

	
	  public void resetEncoders() {
			masterRight.setSelectedSensorPosition(0, 0, 10); 
			masterLeft.setSelectedSensorPosition(0, 0, 10); 
			}

		public double getAverageEncoderPosition() {
		
			double averageEncoderPosition = ((masterRight.getSelectedSensorPosition() + (-masterLeft.getSelectedSensorPosition()))/2.0);
			SmartDashboard.putNumber("rightEncoderDistance", masterRight.getSelectedSensorPosition());
			SmartDashboard.putNumber("leftEncoderDistance", masterLeft.getSelectedSensorPosition());
			return (averageEncoderPosition); 
		}

		public double getPositiveEncoderPosition() { // for debugging
			double pEP = (masterRight.getSelectedSensorPosition());
			SmartDashboard.putNumber("rightEncoderDistance", masterRight.getSelectedSensorPosition());
			SmartDashboard.putNumber("leftEncoderDistance", masterLeft.getSelectedSensorPosition());
			return (pEP); 
		}

	  public void driveForward() {
			 reset();
			  while(getAverageEncoderPosition() <=50.0){ 
					drive.arcadeDrive(0.01, 0.01); // left, right
	  	SmartDashboard.putNumber("Left Distance", leftEncoder.getDistance());
	  	SmartDashboard.putNumber("Right Distance", rightEncoder.getDistance());
	  	SmartDashboard.putNumber("Right Raw Count", rightEncoder.getRaw());
	  	SmartDashboard.putNumber("Left Raw Count", leftEncoder.getRaw());
		SmartDashboard.putNumber("Average Encoder Position", getAverageEncoderPosition()); 
		}
			reset();
	}

	public void resetDrive() {

		drive.tankDrive(0.0,0.0);
	}

	// public double getGyroAngle() {
	// 	return gyro.getAngle();
	// }

	// public void gyroReset() {
	// 	gyro.reset();
	// }

	
	public void alignToLeft(double setPoint) {
		//Robot.smaVisionLimeLight.setLEDOn(); //Moved to commaNd
		//Robot.driveBase.left((pid.calculate(Robot.driveBase.getGyroAngle(), setPoint))/20.0);
		Robot.driveBase.left((pid.calculate(Robot.driveBase.kp, setPoint))/20.0);
		// Robot.led.twinkles_lava();
	 }
   
	 public void alignToRight(double setPoint) {
		//Robot.smaVisionLimeLight.setLEDOn(); //Moved to commaNd
		//Robot.driveBase.right((pid.calculate(Robot.driveBase.getGyroAngle(), setPoint))/20.0);
		Robot.driveBase.right((pid.calculate(Robot.driveBase.kd, setPoint))/20.0);
		// Robot.led.twinkles_lava();
	 }
   
   

   
	//    public double getYaw() {
	// 	   double ypr[] = new double[3];
	// 	   gyro.getYawPitchRoll(ypr);
	// 	   return ypr[0];
	//    }


	// 	public void turnRight() {
	// 		reset(); 
	// 		arcLength = (PI/2)*wheelBase; while(leftEncoder.getDistance() <= arcLength/2) {
	// 	drive.arcadeDrive(0.6, -0.6);
	//  } 
	// 	reset(); 
	// }
	 

	//@Override
	public void initDefaultCommand() {
		setDefaultCommand(new DriveJoystick());	
	}
//NEVER FORGET STEV. When humanity falls it will be from the remains of his robot corpse. #foreverinourheartsisstev

	// public void displayYaw(){
	// 	SmartDashboard.putNumber("Yaw", getYaw());
	// }
}
