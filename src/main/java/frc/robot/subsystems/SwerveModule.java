package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ModuleConstants;
import frc.robot.utils.ShuffleboardContent;
import frc.robot.utils.Enums.ModulePosition;

public class SwerveModule extends SubsystemBase {
    public final CANSparkMax m_driveSparkMax;
    public final CANSparkMax m_turnSparkMax;
    public final RelativeEncoder m_driveEncoder;
    public final CANcoder m_turnCANcoder;
    public final RelativeEncoder m_turnEncoder;
    public final double m_turnEncoderOffsetDeg;
    public final ModulePosition m_modulePosition;

    public boolean m_driveMotorConnected;
    public boolean m_turnMotorConnected;
    public boolean m_turnCoderConnected;
    public SwerveModuleState m_state;

    private final SparkPIDController m_drivePIDController;
    private final SparkPIDController m_turnPIDController;

    public SwerveModule(
            ModulePosition modulePosition,
            int driveMotorCanChannel,
            int turnMotorCanChannel,
            int cancoderCanChannel,
            boolean driveMotorInverted,
            boolean turnMotorInverted,
            double turnEncoderOffsetDeg) {

        m_modulePosition = modulePosition;

        m_driveSparkMax = new CANSparkMax(driveMotorCanChannel, MotorType.kBrushless);
        m_turnSparkMax = new CANSparkMax(turnMotorCanChannel, MotorType.kBrushless);

        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        m_driveSparkMax.restoreFactoryDefaults();
        m_turnSparkMax.restoreFactoryDefaults();

        // turn absolute encoder setup
        m_turnCANcoder = new CANcoder(cancoderCanChannel);
        m_turnCANcoder.getConfigurator().apply(new CANcoderConfiguration());

        // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
        m_driveEncoder = m_driveSparkMax.getEncoder();
        m_turnEncoder = m_turnSparkMax.getEncoder();
        m_drivePIDController = m_driveSparkMax.getPIDController();
        m_turnPIDController = m_turnSparkMax.getPIDController();
        m_drivePIDController.setFeedbackDevice(m_driveEncoder);
        m_turnPIDController.setFeedbackDevice(m_turnEncoder);

        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.
        m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderPositionFactor);
        m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderVelocityFactor);

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        m_turnEncoder.setPositionConversionFactor(ModuleConstants.kTurnEncoderPositionFactor);
        m_turnEncoder.setVelocityConversionFactor(ModuleConstants.kTurnEncoderVelocityFactor);

        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.
        m_driveEncoder.setInverted(driveMotorInverted);
        m_turnEncoder.setInverted(turnMotorInverted);

        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        m_turnPIDController.setPositionPIDWrappingEnabled(true);
        m_turnPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurnEncoderPositionPIDMinInput);
        m_turnPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurnEncoderPositionPIDMaxInput);

        // Set the PID gains for the driving motor. Note these are example gains, and
        // you
        // may need to tune them for your own robot!
        m_drivePIDController.setP(ModuleConstants.kDriveP);
        m_drivePIDController.setI(ModuleConstants.kDriveI);
        m_drivePIDController.setD(ModuleConstants.kDriveD);
        m_drivePIDController.setFF(ModuleConstants.kDriveFF);
        m_drivePIDController.setOutputRange(ModuleConstants.kDriveMinOutput,
                ModuleConstants.kDriveMaxOutput);

        // Set the PID gains for the turning motor. Note these are example gains, and
        // you
        // may need to tune them for your own robot!
        m_turnPIDController.setP(ModuleConstants.kTurnP);
        m_turnPIDController.setI(ModuleConstants.kTurnI);
        m_turnPIDController.setD(ModuleConstants.kTurnD);
        m_turnPIDController.setFF(ModuleConstants.kTurnFF);
        m_turnPIDController.setOutputRange(ModuleConstants.kTurnMinOutput,
                ModuleConstants.kTurnMaxOutput);

        m_driveSparkMax.setIdleMode(ModuleConstants.kDriveMotorIdleMode);
        m_turnSparkMax.setIdleMode(ModuleConstants.kTurnMotorIdleMode);
        m_driveSparkMax.setSmartCurrentLimit(ModuleConstants.kDriveMotorCurrentLimit);
        m_turnSparkMax.setSmartCurrentLimit(ModuleConstants.kTurnMotorCurrentLimit);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        m_driveSparkMax.burnFlash();
        m_turnSparkMax.burnFlash();

        m_turnEncoderOffsetDeg = turnEncoderOffsetDeg;
        m_state.angle = new Rotation2d(m_turnCANcoder.getAbsolutePosition().getValueAsDouble() - m_turnEncoderOffsetDeg);
        m_driveEncoder.setPosition(0);

        checkCAN();

        ShuffleboardContent.initDriveShuffleboard(this);
        ShuffleboardContent.initTurnShuffleboard(this);
        ShuffleboardContent.initCANCoderShuffleboard(this);
        ShuffleboardContent.initBooleanShuffleboard(this);
        ShuffleboardContent.initCoderBooleanShuffleboard(this);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.minus(Rotation2d.fromDegrees(m_turnEncoderOffsetDeg)); // changed plus to minus & Radian to Degrees (Macy)

        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
                new Rotation2d(m_turnCANcoder.getAbsolutePosition().getValueAsDouble() - m_turnEncoderOffsetDeg)); // changed from m_turnEncoder (Macy)

        // Command driving and turning SPARKS MAX towards their respective setpoints.
        m_drivePIDController.setReference(optimizedDesiredState.speedMetersPerSecond,
                CANSparkMax.ControlType.kVelocity);
        m_turnPIDController.setReference(optimizedDesiredState.angle.getRadians(),
                CANSparkMax.ControlType.kPosition);

        m_state = optimizedDesiredState; // changed desiredState to optimizedDesiredState (Macy)
    }

    public SwerveModulePosition getPosition() {
      // Apply chassis angular offset to the encoder position to get the position
      // relative to the chassis.
      return new SwerveModulePosition(
          m_driveEncoder.getPosition(),
          new Rotation2d(m_turnCANcoder.getAbsolutePosition().getValueAsDouble() - m_turnEncoderOffsetDeg));
    }
    
    private boolean checkCAN() {
        m_driveMotorConnected = m_driveSparkMax.getFirmwareVersion() != 0;
        m_turnMotorConnected = m_turnSparkMax.getFirmwareVersion() != 0;
        m_turnCoderConnected = m_turnCANcoder.getDeviceID() != 0;

        return m_driveMotorConnected && m_turnMotorConnected && m_turnCoderConnected;
    }
}
