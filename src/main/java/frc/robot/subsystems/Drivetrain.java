// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    // The motors on the left side of the drive.
    private final CANSparkMax leftMotor1 = new CANSparkMax(DriveConstants.LEFT_MOTOR1_PORT, MotorType.kBrushless);
    private final CANSparkMax leftMotor2 = new CANSparkMax(DriveConstants.LEFT_MOTOR2_PORT, MotorType.kBrushless);
    private final CANSparkMax rightMotor1 = new CANSparkMax(DriveConstants.RIGHT_MOTOR1_PORT, MotorType.kBrushless);
    private final CANSparkMax rightMotor2 = new CANSparkMax(DriveConstants.RIGHT_MOTOR2_PORT, MotorType.kBrushless);

    private final MotorControllerGroup leftMotors = new MotorControllerGroup(
            leftMotor1,
            leftMotor2);

    // The motors on the right side of the drive.
    private final MotorControllerGroup rightMotors = new MotorControllerGroup(
            rightMotor1,
            rightMotor2);

    // The robot's drive
    private final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

    // The left-side drive encoder
    private final RelativeEncoder leftEncoder = leftMotor1.getEncoder();

    // The right-side drive encoder
    private final RelativeEncoder rightEncoder = rightMotor1.getEncoder();

    // The gyro sensor
    private final ADIS16470_IMU gyro = new ADIS16470_IMU();

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry odometry;

    /** Creates a new DriveSubsystem. */
    public Drivetrain() {
        // Define variables

        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        rightMotors.setInverted(true);

        // Sets the distance per pulse for the encoders
        leftEncoder.setPositionConversionFactor(DriveConstants.ENCODER_POSITION_CONVERSION_FACTOR);
        rightEncoder.setPositionConversionFactor(DriveConstants.ENCODER_POSITION_CONVERSION_FACTOR);

        resetEncoders();
        odometry = new DifferentialDriveOdometry(
                getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        odometry.update(
                getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(
                getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), pose);
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void drive(double fwd, double rot) {
        drive.arcadeDrive(fwd, rot);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotors.setVoltage(leftVolts);
        rightMotors.setVoltage(rightVolts);
        drive.feed();
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
    }

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public RelativeEncoder getLeftEncoder() {
        return leftEncoder;
    }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public RelativeEncoder getRightEncoder() {
        return rightEncoder;
    }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more
     * slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        drive.setMaxOutput(maxOutput);
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     * 
     * @return a Rotation2d representing the robot's heading
     */
    public Rotation2d getRotation2d() {
        return new Rotation2d(gyro.getAngle());
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return getRotation2d().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return -gyro.getRate();
    }

    /**
     * Schedules the config commands to run. This should be run every time the robot
     * is disabled.
     */
    public void scheduleConfigCommands() {
        CommandScheduler.getInstance().schedule(new InstantCommand(() -> {
            leftMotor1.restoreFactoryDefaults();
            leftMotor2.restoreFactoryDefaults();
            rightMotor1.restoreFactoryDefaults();
            rightMotor2.restoreFactoryDefaults();
        }).ignoringDisable(true));
    }

    /**
     * Resets the relative rotation encoders to currently read a position of 0.
     */
    public void resetRelativeRotationEncoders() {
        CommandScheduler.getInstance().schedule(
                new InstantCommand(() -> resetEncoders()).ignoringDisable(false));
    }

}