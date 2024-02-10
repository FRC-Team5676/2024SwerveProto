package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;
import frc.robot.utils.Enums.ModulePosition;
import frc.robot.utils.ShuffleboardContent;

public class SwerveDrive extends SubsystemBase {
    public static final SwerveDriveKinematics m_driveKinematics = new SwerveDriveKinematics(
            new Translation2d(DriveConstants.kRobotWidth / 2, DriveConstants.kRobotLength / 2), // Front Left
            new Translation2d(DriveConstants.kRobotWidth / 2, -DriveConstants.kRobotLength / 2), // Front Right
            new Translation2d(-DriveConstants.kRobotWidth / 2, DriveConstants.kRobotLength / 2), // Rear Left
            new Translation2d(-DriveConstants.kRobotWidth / 2, -DriveConstants.kRobotLength / 2)); // Rear Right

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

    // Odometry class for tracking robot pose
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
            m_driveKinematics,
            Rotation2d.fromDegrees(getYaw()),
            new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_rearLeft.getPosition(),
                    m_rearRight.getPosition()
            });

    public SwerveDrive() {
        ShuffleboardContent.initGyro(this);
        configAuto();
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block

        // Update the odometry in the periodic block
        m_odometry.update(
                Rotation2d.fromDegrees(getYaw()),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                });
    }

    public void drive(double throttle, double strafe, double rotation) {

        // Convert the commanded speeds into the correct units for the drivetrain
        double throttleSpeed = throttle * DriveConstants.kMaxSpeedMetersPerSecond;
        double strafeSpeed = strafe * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotationSpeed = rotation * DriveConstants.kMaxRotationRadiansPerSecond;

        SwerveModuleState[] swerveModuleStates = m_driveKinematics.toSwerveModuleStates(
                m_fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(throttleSpeed, strafeSpeed, rotationSpeed,
                                Rotation2d.fromDegrees(getYaw()))
                        : new ChassisSpeeds(throttleSpeed, strafeSpeed, rotationSpeed));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    public void zeroGyro() {
        m_gyro.zeroYaw();
    }

    // Yaw Z Direction
    // Perpendicular to board
    // Positive towards the ceiling
    // + Clockwise / - Counter-clockwise
    public double getYaw() {
        // if (m_gyro.getYaw() < 0) return m_gyro.getYaw() + 360;
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

    private Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    private void resetPose(Pose2d pose2d) {
        SwerveModuleState frontLeftState = m_frontLeft.getSwerveModuleState();
        SwerveModuleState frontRightState = m_frontRight.getSwerveModuleState();
        SwerveModuleState rearLeftState = m_rearLeft.getSwerveModuleState();
        SwerveModuleState rearRightState = m_rearRight.getSwerveModuleState();
        SwerveModulePosition[] modulePositions = {
            new SwerveModulePosition(0, frontLeftState.angle),
            new SwerveModulePosition(0, frontRightState.angle),
            new SwerveModulePosition(0, rearLeftState.angle),
            new SwerveModulePosition(0, rearRightState.angle)
        };

        m_odometry = new SwerveDriveOdometry(m_driveKinematics, Rotation2d.fromDegrees(getYaw()), modulePositions, pose2d);
    }

    private ChassisSpeeds getRobotRelativeSpeeds() {
        SwerveModuleState frontLeftState = m_frontLeft.getSwerveModuleState();
        SwerveModuleState frontRightState = m_frontRight.getSwerveModuleState();
        SwerveModuleState rearLeftState = m_rearLeft.getSwerveModuleState();
        SwerveModuleState rearRightState = m_rearRight.getSwerveModuleState();

        // Convert to chassis speeds
        ChassisSpeeds chassisSpeeds = m_driveKinematics.toChassisSpeeds(
            frontLeftState, frontRightState, rearLeftState, rearRightState);

        return chassisSpeeds;
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {

        SwerveModuleState[] swerveModuleStates = m_driveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    private void configAuto() {
        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

    }
}