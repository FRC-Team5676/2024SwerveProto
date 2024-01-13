package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ModuleConstants;
import frc.robot.utils.ShuffleboardContent;
import frc.robot.utils.Enums.ModulePosition;

public class SwerveModule extends SubsystemBase {
    public final CANSparkMax m_driveMotor;
    public final CANSparkMax m_turnMotor;
    public final RelativeEncoder m_driveEncoder;
    public final CANCoder m_turnCANcoder;
    public final RelativeEncoder m_turnEncoder;
    public final double m_turnEncoderOffset;
    public final ModulePosition m_modulePosition;

    public boolean m_driveMotorConnected;
    public boolean m_turnMotorConnected;
    public boolean m_turnCoderConnected;
    public SwerveModuleState m_state;

    private final SparkMaxPIDController m_driveVelController;
    private final SparkMaxPIDController m_turnPosController;

    public SwerveModule(
            ModulePosition modulePosition,
            int driveMotorCanChannel,
            int turnMotorCanChannel,
            int cancoderCanChannel,
            boolean driveMotorReversed,
            boolean turnMotorReversed,
            double turnEncoderOffset) {

        m_modulePosition = modulePosition;

        // Drive Motor setup
        m_driveMotor = new CANSparkMax(driveMotorCanChannel, MotorType.kBrushless);
        m_driveMotor.restoreFactoryDefaults();
        m_driveMotor.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
        m_driveMotor.enableVoltageCompensation(ModuleConstants.kVoltCompensation);
        m_driveMotor.setInverted(driveMotorReversed);
        m_driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // turn motor setup
        m_turnMotor = new CANSparkMax(turnMotorCanChannel, MotorType.kBrushless);
        m_turnMotor.restoreFactoryDefaults();
        m_turnMotor.setSmartCurrentLimit(ModuleConstants.kTurnMotorCurrentLimit);
        m_turnMotor.enableVoltageCompensation(ModuleConstants.kVoltCompensation);
        m_turnMotor.setInverted(turnMotorReversed);
        m_turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // drive encoder setup
        m_driveEncoder = m_driveMotor.getEncoder();
        m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveMetersPerEncRev);
        m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kModuleFreeSpeedMetersPerSec);

        // turn encoder setup
        m_turnCANcoder = new CANCoder(cancoderCanChannel);
        m_turnCANcoder.configFactoryDefault();
        m_turnCANcoder.configAllSettings(GenerateCanCoderConfig());

        m_turnEncoder = m_turnMotor.getEncoder();
        m_turnEncoder.setPositionConversionFactor(ModuleConstants.kTurnDegreesPerMotorRev);
        m_turnEncoder.setVelocityConversionFactor(ModuleConstants.kTurnDegreesPerSec);
        m_turnEncoderOffset = turnEncoderOffset;
        m_turnEncoder.setPosition(m_turnCANcoder.getAbsolutePosition() - m_turnEncoderOffset);

        // Drive PIDF Setup
        m_driveVelController = m_driveMotor.getPIDController();
        m_driveVelController.setP(.01);
        m_driveVelController.setI(0);
        m_driveVelController.setD(0);
        m_driveVelController.setFF(1);

        // Turn PIDF Setup
        m_turnPosController = m_turnMotor.getPIDController();
        m_turnPosController.setP(.01);
        m_turnPosController.setI(0);
        m_turnPosController.setD(0);
        m_turnPosController.setFF(1);

        checkCAN();

        ShuffleboardContent.initDriveShuffleboard(this);
        ShuffleboardContent.initTurnShuffleboard(this);
        ShuffleboardContent.initCANCoderShuffleboard(this);
        ShuffleboardContent.initBooleanShuffleboard(this);
        ShuffleboardContent.initCoderBooleanShuffleboard(this);
    }

    public void setDesiredState(SwerveModuleState desiredState) {

        // Save State
        m_state = desiredState;

        // Set Position
        m_turnPosController.setReference(m_state.angle.getDegrees(), CANSparkMax.ControlType.kPosition);
        m_driveVelController.setReference(m_state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    }

    private static CANCoderConfiguration GenerateCanCoderConfig() {
        CANCoderConfiguration sensorConfig = new CANCoderConfiguration();

        sensorConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        sensorConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        sensorConfig.sensorTimeBase = SensorTimeBase.PerSecond;

        return sensorConfig;
    }

    private boolean checkCAN() {
        m_driveMotorConnected = m_driveMotor.getFirmwareVersion() != 0;
        m_turnMotorConnected = m_turnMotor.getFirmwareVersion() != 0;
        m_turnCoderConnected = m_turnCANcoder.getFirmwareVersion() > 0;

        return m_driveMotorConnected && m_turnMotorConnected && m_turnCoderConnected;
    }
}
