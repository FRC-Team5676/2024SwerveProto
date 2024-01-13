package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;
import frc.robot.utils.Enums.ModulePosition;

public class SwerveDrive extends SubsystemBase {
    public static final SwerveDriveKinematics m_driveKinematics = new SwerveDriveKinematics(
            new Translation2d(-DriveConstants.kRobotWidth / 2, DriveConstants.kRobotLength / 2), // Front Left
            new Translation2d(DriveConstants.kRobotWidth / 2, DriveConstants.kRobotLength / 2), // Front Right
            new Translation2d(-DriveConstants.kRobotWidth / 2, -DriveConstants.kRobotLength / 2), // Rear Left
            new Translation2d(DriveConstants.kRobotWidth / 2, -DriveConstants.kRobotLength / 2)); // Rear Right

    private final SwerveModule m_frontLeft = new SwerveModule(
            ModulePosition.FRONT_LEFT,
            DriveConstants.kFrontLeftDriveMotorCanId,
            DriveConstants.kFrontLeftTurnMotorCanId,
            DriveConstants.kFrontLeftTurnEncoderCanId,
            DriveConstants.kFrontLeftDriveMotorReversed,
            DriveConstants.kFrontLeftTurningMotorReversed,
            DriveConstants.kFrontLeftAngularOffset);

    private final SwerveModule m_frontRight = new SwerveModule(
            ModulePosition.FRONT_RIGHT,
            DriveConstants.kFrontRightDriveMotorCanId,
            DriveConstants.kFrontRightTurnMotorCanId,
            DriveConstants.kFrontRightTurnEncoderCanId,
            DriveConstants.kFrontRightDriveMotorReversed,
            DriveConstants.kFrontRightTurningMotorReversed,
            DriveConstants.kFrontRightAngularOffset);

    private final SwerveModule m_rearLeft = new SwerveModule(
            ModulePosition.REAR_LEFT,
            DriveConstants.kRearLeftDriveMotorCanId,
            DriveConstants.kRearLeftTurnMotorCanId,
            DriveConstants.kRearLeftTurnEncoderCanId,
            DriveConstants.kRearLeftDriveMotorReversed,
            DriveConstants.kRearLeftTurnMotorReversed,
            DriveConstants.kRearLeftAngularOffset);

    private final SwerveModule m_rearRight = new SwerveModule(
            ModulePosition.REAR_RIGHT,
            DriveConstants.kRearRightDriveMotorCanId,
            DriveConstants.kRearRightTurnMotorCanId,
            DriveConstants.kRearRightTurnEncoderCanId,
            DriveConstants.kRearRightDriveMotorReversed,
            DriveConstants.kRearRightTurnMotorReversed,
            DriveConstants.kRearRightAngularOffset);

    private final AHRS m_gyro;

    private ChassisSpeeds m_currentChassisSpeeds;
    private boolean m_fieldRelative = DriveConstants.kFieldRelative;

    public SwerveDrive() {
        m_gyro = new AHRS(SPI.Port.kMXP);
        m_currentChassisSpeeds = new ChassisSpeeds();
    }

    public void drive(double throttle, double strafe, double rotation) {

        throttle *= DriveConstants.kMaxSpeedMetersPerSecond;
        strafe *= DriveConstants.kMaxSpeedMetersPerSecond;
        rotation *= DriveConstants.kMaxRotationRadiansPerSecond;

        ChassisSpeeds chassisSpeeds = m_fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        throttle, strafe, rotation, Rotation2d.fromDegrees(getYaw()))
                : new ChassisSpeeds(throttle, strafe, rotation);

        SwerveModuleState[] desiredModuleStates = m_driveKinematics.toSwerveModuleStates(chassisSpeeds);

        setModuleStates(desiredModuleStates);
    }

    public void zeroGyro() {
        m_gyro.zeroYaw();
    }

    // Yaw Z Direction
    // Perpendicular to board
    // Positive towards the ceiling
    // + Clockwise / - Counter-clockwise
    public double getYaw() {
        return m_gyro.getYaw();
    }

    // Roll Y direction
    // Width (short side) or board
    // Positive towards left when looking towards the RoboRio connector
    // + Left / - Right
    public double getRoll() {
        return m_gyro.getRoll();
    }

    // Pitch X direction
    // lengthwise (long side) of board
    // Positive towards RoboRio connector
    // + Forward / - Backwards
    public double getPitch() {
        return m_gyro.getPitch();
    }

    private void setModuleStates(SwerveModuleState[] desiredModuleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, m_currentChassisSpeeds,
                DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxSpeedMetersPerSecond,
                DriveConstants.kMaxRotationRadiansPerSecond);

        m_frontLeft.setDesiredState(desiredModuleStates[0]);
        m_frontRight.setDesiredState(desiredModuleStates[1]);
        m_rearLeft.setDesiredState(desiredModuleStates[2]);
        m_rearRight.setDesiredState(desiredModuleStates[3]);

        m_currentChassisSpeeds = m_driveKinematics.toChassisSpeeds(desiredModuleStates);
    }

    public void toggleFieldRelative() {
        m_fieldRelative = !m_fieldRelative;
    }
}