package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public final class DriveConstants {
    public static final double kXYDeadband = 0.05;
    public static final double kRotationDeadband = 0.1;

    public static final boolean kTurnMotorInverted = true;

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = 0;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kRearLeftChassisAngularOffset = 0;
    public static final double kRearRightChassisAngularOffset = 0;

    public static final double kFrontLeftAngularOffset = 302.9;
    public static final double kFrontRightAngularOffset = 308.2;
    public static final double kRearLeftAngularOffset = 321.1;
    public static final double kRearRightAngularOffset = 187.0;

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
    public static final double kMaxRotationRadiansPerSecond = Math.PI; // radians per second

    // Chassis configuration
    public static final double kRobotWidth = Units.inchesToMeters(21.25);  // Distance between centers of right and left wheels on robot
    public static final double kRobotLength = Units.inchesToMeters(21.25); // Distance between front and back wheels on robot

    // Drive Control
    public static final boolean kFieldRelative = true;
    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)
}
