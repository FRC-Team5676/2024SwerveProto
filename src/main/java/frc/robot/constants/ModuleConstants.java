package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public final class ModuleConstants {
    public static final String[] modAbrev = { "_FL", "_FR", "_RL", "_RR" };

    public static final int kDrivingMotorCurrentLimit = 20; // amps
    public static final int kTurnMotorCurrentLimit = 20; // amps
    public static final double kVoltCompensation = 12.0;

    // From: https://www.swervedrivespecialties.com/products/mk4i-swerve-module
    /* Drive */
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
    public static final double kMk4iL1DriveGearRatio = 8.14;
    public static final double kDriveMetersPerEncRev = (kWheelDiameterMeters * Math.PI) / kMk4iL1DriveGearRatio;
    public static final double kModuleFreeSpeedMetersPerSec = Units.feetToMeters(12.5);
    /* Turn */
    public static final double kMk4iL1TurnGearRatio = 150/7;
    public static final double kTurnDegreesPerMotorRev = 360 / kMk4iL1TurnGearRatio;
    public static final int kMotorFreeSpeedRpm = 5820;
    public static final double kTurnDegreesPerSec = kTurnDegreesPerMotorRev * kMotorFreeSpeedRpm / 60;
}
