// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /*
     * TAKE NOTE: This is a template file. It is intended to be copied and renamed
     * for each new robot and the constants adjusted as needed. It is not intended
     * to be used directly as the Constants class for a robot.
     */
    public final static class ControllerConstants {
      public static final int DRIVER_CONTROLLER_PORT = 0;
      public static final int OPERATOR_CONTROLLER_PORT = 1;

      public static final double DRIVER_DEADBAND_FORWARD = 0.05;
      public static final double DRIVER_DEADBAND_TURN = 0.03;

      public static final double BETA = 2;
      public static final double ZETA = 0.7;

      public static final double RAMSETE_KP = 1;
      public static final double RAMSETE_KI = 0;
      public static final double REMSETE_KD = 0;
      public static final double DRIVER_DEADZONE = 0.1;
  }

  public final static class DrivetrainConstants {

      public static final double WHEEL_DIAMETER_METERS = 0.15;
      public static final double WHEEL_CIRC_METERS = WHEEL_DIAMETER_METERS * Math.PI;
      public static final double DRIVETRAIN_GEAR_RATIO = 5.95;
      public static final double DRIVING_ENCODER_POS_FACTOR = WHEEL_CIRC_METERS / DRIVETRAIN_GEAR_RATIO; // meters
      public static final double DRIVING_ENCODER_VEL_FACTOR = DRIVING_ENCODER_POS_FACTOR / 60; // m/s

      public static final double DRIVING_SPEED_MULTIPLIER = 1;
      //TODO: Tune this value
      public static final double ks_VOLTS = 0.22;
      public static final double kv_VOLT_SECONDS_PER_METER = 1.98;
      public static final double ka_VOLT_SECONDS_SQUARED_PER_METER = 0.2;

      // Example value only - as above, this must be tuned for your drive!
      //TODO: Tune this value
      public static final double kp_DRIVE_VEL = 8.5;
      
      //TODO: Tune these values
      private static final double kv_VOLT_SECONDS_PER_RADIAN = 1.5;
      private static final double ka_VOLT_SECONDS_SQUARED_PER_RADIAN = 0.3;
      
      public static final double GEAR_RATIO = 5.95;

      public static final int LEFT_MOTOR_FRONT_CAN_ID = 1;
      public static final int LEFT_MOTOR_FOLLOWER_CAN_ID = 2;

      public static final int RIGHT_MOTOR_FRONT_CAN_ID = 3;
      public static final int RIGHT_MOTOR_FOLLOWER_CAN_ID = 4;
      public static final LinearSystem<N2, N2, N2> DRIVETRIAN_PLANT = 
            LinearSystemId.identifyDrivetrainSystem(
                kv_VOLT_SECONDS_PER_METER,
                ka_VOLT_SECONDS_SQUARED_PER_METER,
                kv_VOLT_SECONDS_PER_RADIAN,
                ka_VOLT_SECONDS_SQUARED_PER_RADIAN
            );
      public static final DCMotor GEARBOX = DCMotor.getNEO(2);

      // These bools control if the drivetrain is flipped or not on a side
      // change these if one is not moving the right direction
      public static final boolean RIGHT_MOTOR_INVERT = true;
      public static final boolean LEFT_MOTOR_INVERT = false;

      // These bools are based upon the gearbox, and will not be the issue if
      // there are inversions in the drivetrain
      public static final boolean RIGHT_FOLLOWER_INVERTED = false;
      public static final boolean LEFT_FOLLOWER_INVERTED = false;

      // TODO: Use these values
      public static final double TURNING_P = 0.6;
      public static final double TURNING_I = 0;
      public static final double TURNING_D = 0.1;

      public static final double DRIVING_P = 0.7;
      public static final double DRIVING_I = 0;
      public static final double DRIVING_D = 0.1;

      public static final double SLEW_RATE_TURN_NEGATIVE = -5;
      public static final double SLEW_RATE_TURN_POSITIVE = 5;
      public static final double SLEW_RATE_DRIVE_POSITIVE = 5;
      public static final double SLEW_RATE_DRIVE_NEGATIVE = -5;

      // TODO: Tune these values
      public static final int DRIVE_TO_DISTANCE_TOLERANCE = 4;
      public static final double ANGLE_TOLERANCE = 0.1;

      public static final double TRACK_WIDTH_METERS = 0.5588;

      public static final double MAX_DRIVE_VELOCITY = 4;
      public static final double MAX_DRIVE_ACCELERATION = 1.5;

      public static final double MAX_DRIVE_VOLTAGE = 7;

      public static final double kS = 0.0;
      public static final double kV = 0.0;
      public static final double kA = 0.0;

      public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
              DrivetrainConstants.TRACK_WIDTH_METERS);

      public static final double MAX_DRIVE_SPEED = 0.8;
      public static final boolean ENCODER_REVERSED = false;
        public static final double ENCODER_RESOLUTION = 42;
  }
}