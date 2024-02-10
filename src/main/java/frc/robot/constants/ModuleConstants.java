package frc.robot.constants;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.util.Units;

public final class ModuleConstants {
    public static final double kNeoFreeSpeedRpm = 5676;

    public static final String[] modAbrev = { "_FL", "_FR", "_RL", "_RR" };

    public static final double kVoltCompensation = 12.0; // volts
    public static final int kDriveMotorCurrentLimit = 50; // amps
    public static final int kTurnMotorCurrentLimit = 20; // amps

    // From: https://www.swervedrivespecialties.com/products/mk4i-swerve-module
    /* Drive */
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
    public static final double kMk4iL1DriveGearRatio = 8.14;
    public static final double kDriveMotorFreeSpeedRps = kNeoFreeSpeedRpm / 60;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    public static final double kDriveWheelFreeSpeedRps = (kDriveMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kMk4iL1DriveGearRatio;

    public static final double kDriveEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kMk4iL1DriveGearRatio; // meters
    public static final double kDriveEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kMk4iL1DriveGearRatio) / 60.0; // meters per second

    public static final double kDriveP = 0.04;
    public static final double kDriveI = 0;
    public static final double kDriveD = 0;
    public static final double kDriveFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDriveMinOutput = -1;
    public static final double kDriveMaxOutput = 1;

    public static final IdleMode kDriveMotorIdleMode = IdleMode.kBrake;

    // Invert the Drive encoder, since the output shaft rotates in the opposite direction of
    // the drive motor in the mk4i module.
    public static final boolean kDriveEncoderInverted = true;


    /* Turn */
    public static final double kMk4iL1TurnGearRatio = 150/7;
    public static final double kTurnEncoderPositionFactor = (2 * Math.PI)
        / kMk4iL1TurnGearRatio; // added gear ratio (E. Macy)
    public static final double kTurnEncoderVelocityFactor = (2 * Math.PI) 
        / kMk4iL1TurnGearRatio / 60.0; // added gear ratio (E. Macy)

    public static final double kTurnEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurnEncoderPositionPIDMaxInput = kTurnEncoderPositionFactor; // radians

    public static final double kTurnP = 1; //0.4 // -5
    public static final double kTurnD = 0; //0
    public static final double kTurnI = 0; //0
    public static final double kTurnFF = 0; //1 / kMk4iL1TurnGearRatio;
    public static final double kTurnMinOutput = -1;
    public static final double kTurnMaxOutput = 1;

    public static final IdleMode kTurnMotorIdleMode = IdleMode.kBrake;

    // Invert the Turn encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the mk4i module.
    public static final boolean kTurnEncoderInverted = true;
}
