package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public final class DriveConstants {
    public static final boolean kDriveMotorReversed = true;
    public static final boolean kTurnMotorReversed = true;

    public static final double kFrontLeftAngularOffset = 48.4;
    public static final double kFrontRightAngularOffset = -7.5;
    public static final double kRearLeftAngularOffset = -6.4;
    public static final double kRearRightAngularOffset = 30.1;

    // Spark MAX Drive Motor CAN IDs
    public static final int kFrontLeftDriveMotorCanId = 32;
    public static final int kFrontRightDriveMotorCanId = 31;
    public static final int kRearLeftDriveMotorCanId = 33;
    public static final int kRearRightDriveMotorCanId = 34;

    // Spark MAX Turn Motor CAN IDs
    public static final int kFrontLeftTurnMotorCanId = 22;
    public static final int kFrontRightTurnMotorCanId = 21;
    public static final int kRearLeftTurnMotorCanId = 23;
    public static final int kRearRightTurnMotorCanId = 24;

    // Spark MAX Turn Encoder CAN IDs
    public static final int kFrontLeftTurnEncoderCanId = 42;
    public static final int kFrontRightTurnEncoderCanId = 41;
    public static final int kRearLeftTurnEncoderCanId = 43;
    public static final int kRearRightTurnEncoderCanId = 44;

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxRotationRadiansPerSecond = 2 * Math.PI; // radians per second


    // Chassis configuration
    public static final double kRobotWidth = Units.inchesToMeters(21.25);  // Distance between centers of right and left wheels on robot
    public static final double kRobotLength = Units.inchesToMeters(21.25); // Distance between front and back wheels on robot

    // Drive Control
    public static final boolean kFieldRelative = true;
    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)
}
