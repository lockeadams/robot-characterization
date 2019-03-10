/**
 * This is a very simple robot program that can be used to send telemetry to
 * the data_logger script to characterize your drivetrain. If you wish to use
 * your actual robot code, you only need to implement the simple logic in the
 * autonomousPeriodic function and change the NetworkTables update rate
 *
 * This program assumes that you are using SparkMAX motor controllers
 * and NEO motors.
 */

package dc;

import java.util.function.Supplier;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

	static private double WHEEL_DIAMETER = 0.5;
	static private double ENCODER_PULSE_PER_REV = 1;
	static private double GEAR_RATIO = 15.32;

	Joystick stick;
	DifferentialDrive drive;

    CANSparkMax leftMotor, rightMotor;
    CANEncoder leftEncoder, rightEncoder;

	Supplier<Double> leftEncoderPosition;
	Supplier<Double> leftEncoderRate;
	Supplier<Double> rightEncoderPosition;
	Supplier<Double> rightEncoderRate;

	NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
	NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");

	double priorAutospeed = 0;
	Number[] numberArray = new Number[9];

	@Override
	public void robotInit() {

		stick = new Joystick(0);

        // Motor controllers
        leftMotor = new CANSparkMax(1, MotorType.kBrushless);
        leftMotor.setInverted(false);
        leftMotor.setIdleMode(IdleMode.kBrake);

        CANSparkMax leftSlave1 = new CANSparkMax(2, MotorType.kBrushless);
        leftSlave1.setInverted(false);
        leftSlave1.setIdleMode(IdleMode.kBrake);
        leftSlave1.follow(leftMotor);

        CANSparkMax leftSlave2 = new CANSparkMax(3, MotorType.kBrushless);
        leftSlave2.setInverted(false);
        leftSlave2.setIdleMode(IdleMode.kBrake);
        leftSlave2.follow(leftMotor);

        rightMotor = new CANSparkMax(4, MotorType.kBrushless);
        rightMotor.setInverted(false);
        rightMotor.setIdleMode(IdleMode.kBrake);

        CANSparkMax rightSlave1 = new CANSparkMax(5, MotorType.kBrushless);
        rightSlave1.setInverted(false);
        rightSlave1.setIdleMode(IdleMode.kBrake);
        rightSlave1.follow(leftMotor);

        CANSparkMax rightSlave2 = new CANSparkMax(6, MotorType.kBrushless);
        rightSlave2.setInverted(false);
        rightSlave2.setIdleMode(IdleMode.kBrake);
        rightSlave2.follow(leftMotor);

        // Encoders
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

		//
		// Configure drivetrain movement
		//

		SpeedControllerGroup leftGroup = new SpeedControllerGroup(leftMotor, leftSlave1, leftSlave2);
		SpeedControllerGroup rightGroup = new SpeedControllerGroup(rightMotor, rightSlave1, rightSlave2);

		drive = new DifferentialDrive(leftGroup, rightGroup);
		drive.setDeadband(0);


		//
		// Configure encoder related functions -- getDistance and getrate should return
		// ft and ft/s
		//

		double encoderConstant = (1 / (ENCODER_PULSE_PER_REV * GEAR_RATIO)) * WHEEL_DIAMETER * Math.PI;

		leftEncoderPosition = () -> leftEncoder.getPosition() * encoderConstant;
		leftEncoderRate = () -> leftEncoder.getVelocity() * encoderConstant / 60.0; // native is rpm

		rightEncoderPosition = () -> rightEncoder.getPosition() * encoderConstant;
		rightEncoderRate = () -> rightEncoder.getVelocity() * encoderConstant / 60.0; // native is rpm

		// Reset encoders
		leftEncoder.setPosition(0);
		rightEncoder.setPosition(0);

		// Set the update rate instead of using flush because of a ntcore bug
		// -> probably don't want to do this on a robot in competition
		NetworkTableInstance.getDefault().setUpdateRate(0.010);
	}

	@Override
	public void disabledInit() {
		System.out.println("Robot disabled");
		drive.tankDrive(0, 0);
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void robotPeriodic() {
		// feedback for users, but not used by the control program
		SmartDashboard.putNumber("l_encoder_pos", leftEncoderPosition.get());
		SmartDashboard.putNumber("l_encoder_rate", leftEncoderRate.get());
		SmartDashboard.putNumber("r_encoder_pos", rightEncoderPosition.get());
		SmartDashboard.putNumber("r_encoder_rate", rightEncoderRate.get());
	}

	@Override
	public void teleopInit() {
		System.out.println("Robot in operator control mode");
	}

	@Override
	public void teleopPeriodic() {
		drive.arcadeDrive(-stick.getY(), stick.getX());
	}

	@Override
	public void autonomousInit() {
		System.out.println("Robot in autonomous mode");
	}

	/**
	 * If you wish to just use your own robot program to use with the data logging
	 * program, you only need to copy/paste the logic below into your code and
	 * ensure it gets called periodically in autonomous mode
	 *
	 * Additionally, you need to set NetworkTables update rate to 10ms using the
	 * setUpdateRate call.
	 */
	@Override
	public void autonomousPeriodic() {

		// Retrieve values to send back before telling the motors to do something
		double now = Timer.getFPGATimestamp();

		double leftPosition = leftEncoderPosition.get();
		double leftRate = leftEncoderRate.get();

		double rightPosition = rightEncoderPosition.get();
		double rightRate = rightEncoderRate.get();

		double battery = RobotController.getBatteryVoltage();

		double leftMotorVolts = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
		double rightMotorVolts = rightMotor.getAppliedOutput() * rightMotor.getBusVoltage();

		// Retrieve the commanded speed from NetworkTables
		double autospeed = autoSpeedEntry.getDouble(0);
		priorAutospeed = autospeed;

		// command motors to do things
		drive.tankDrive(autospeed, autospeed, false);

		// send telemetry data array back to NT
		numberArray[0] = now;
		numberArray[1] = battery;
		numberArray[2] = autospeed;
		numberArray[3] = leftMotorVolts;
		numberArray[4] = rightMotorVolts;
		numberArray[5] = leftPosition;
		numberArray[6] = rightPosition;
		numberArray[7] = leftRate;
		numberArray[8] = rightRate;

		telemetryEntry.setNumberArray(numberArray);
	}
}
