package frc.robot.subsystems;

import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ModuleConstants;
import frc.robot.utils.Enums.ModulePosition;
import frc.robot.utils.ShuffleboardContent;

public class SwerveDrive extends SubsystemBase {
    public static final SwerveDriveKinematics m_driveKinematics = new SwerveDriveKinematics(
            new Translation2d(DriveConstants.kRobotWidth / 2, DriveConstants.kRobotLength / 2), // Front Left
            new Translation2d(DriveConstants.kRobotWidth / 2, -DriveConstants.kRobotLength / 2), // Front Right
            new Translation2d(-DriveConstants.kRobotWidth / 2, DriveConstants.kRobotLength / 2), // Rear Left
            new Translation2d(-DriveConstants.kRobotWidth / 2, -DriveConstants.kRobotLength / 2)); // Rear Right

    private static final double m_moduleOffset = new Translation2d(DriveConstants.kRobotWidth / 2,
            DriveConstants.kRobotLength / 2).getNorm();
    private static final HolonomicPathFollowerConfig m_pathFollowerConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(ModuleConstants.kDriveP, ModuleConstants.kDriveI, ModuleConstants.kDriveD,
                    ModuleConstants.kDriveFF), // Translation constants
            new PIDConstants(ModuleConstants.kTurnD, ModuleConstants.kTurnI, ModuleConstants.kTurnD,
                    ModuleConstants.kTurnFF), // Rotation constants
            DriveConstants.kMaxSpeedMetersPerSecond,
            m_moduleOffset, // Drive base radius (distance from center to furthest module)
            new ReplanningConfig());
    private final SwerveModule m_frontLeft = new SwerveModule(
            ModulePosition.FRONT_LEFT,
            DriveConstants.kFrontLeftDriveMotorCanId,
            DriveConstants.kFrontLeftTurnMotorCanId,
            DriveConstants.kFrontLeftTurnEncoderCanId,
            DriveConstants.kTurnMotorInverted,
            DriveConstants.kFrontLeftAngularOffset);
    private final SwerveModule m_frontRight = new SwerveModule(
            ModulePosition.FRONT_RIGHT,
            DriveConstants.kFrontRightDriveMotorCanId,
            DriveConstants.kFrontRightTurnMotorCanId,
            DriveConstants.kFrontRightTurnEncoderCanId,
            DriveConstants.kTurnMotorInverted,
            DriveConstants.kFrontRightAngularOffset);
    private final SwerveModule m_rearLeft = new SwerveModule(
            ModulePosition.REAR_LEFT,
            DriveConstants.kRearLeftDriveMotorCanId,
            DriveConstants.kRearLeftTurnMotorCanId,
            DriveConstants.kRearLeftTurnEncoderCanId,
            DriveConstants.kTurnMotorInverted,
            DriveConstants.kRearLeftAngularOffset);
    private final SwerveModule m_rearRight = new SwerveModule(
            ModulePosition.REAR_RIGHT,
            DriveConstants.kRearRightDriveMotorCanId,
            DriveConstants.kRearRightTurnMotorCanId,
            DriveConstants.kRearRightTurnEncoderCanId,
            DriveConstants.kTurnMotorInverted,
            DriveConstants.kRearRightAngularOffset);
    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
    private boolean m_fieldRelative = DriveConstants.kFieldRelative;
    private SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
            m_driveKinematics,
            Rotation2d.fromDegrees(getAngle()),
            new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_rearLeft.getPosition(),
                    m_rearRight.getPosition()
            });

    private DigitalInput noteSensor = new DigitalInput(0);
    public boolean noteDetected = false;

    public SwerveDrive() {
        SwerveModuleState zeroState = new SwerveModuleState(0, new Rotation2d(0));
        m_frontLeft.setDesiredState(zeroState);
        m_frontRight.setDesiredState(zeroState);
        m_rearLeft.setDesiredState(zeroState);
        m_rearRight.setDesiredState(zeroState);
        
        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetPose,
                this::getSpeeds,
                this::autonDriveRobotRelative,
                m_pathFollowerConfig,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    Optional<Alliance> alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);
        ShuffleboardContent.initGyro(this);
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(Rotation2d.fromDegrees(getAngle()), getPositions());
        noteDetected = noteSensor.get();
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        m_odometry.resetPosition(Rotation2d.fromDegrees(getAngle()), getPositions(), pose);
    }

    public ChassisSpeeds getSpeeds() {
        return m_driveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void teleopDrive(double throttle, double strafe, double rotation) {

        // Convert the commanded speeds into the correct units for the drivetrain
        double throttleSpeed = throttle * DriveConstants.kMaxSpeedMetersPerSecond;
        double strafeSpeed = strafe * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotationSpeed = rotation * DriveConstants.kMaxRotationRadiansPerSecond;

        SwerveModuleState[] swerveModuleStates = m_driveKinematics.toSwerveModuleStates(
                m_fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(throttleSpeed, strafeSpeed, rotationSpeed,
                                Rotation2d.fromDegrees(getYaw()))
                        : new ChassisSpeeds(throttleSpeed, strafeSpeed, rotationSpeed));

        setRobotState(swerveModuleStates);
    }

    public void autonDriveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] swerveModuleStates = m_driveKinematics.toSwerveModuleStates(targetSpeeds);
        setRobotState(swerveModuleStates);
    }

    public void zeroGyro() {
        m_gyro.zeroYaw();
    }

    // Yaw Z Direction
    // Perpendicular to board
    // Positive towards the ceiling
    // + Clockwise / - Counter-clockwise
    public double getYaw() {
        return -m_gyro.getYaw();
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

    public void toggleFieldRelative() {
        m_fieldRelative = !m_fieldRelative;
    }

    private double getAngle() {
        return m_gyro.getAngle();
    }

    private void setRobotState(SwerveModuleState[] swerveModuleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    private SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
        };
    }

    private SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_rearLeft.getState(),
                m_rearRight.getState()
        };
    }
}