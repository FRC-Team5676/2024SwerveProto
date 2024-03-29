package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
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
        public final double m_turnAngleCorrectionRad;
        public final ModulePosition m_modulePosition;

        public boolean m_driveMotorConnected;
        public boolean m_turnMotorConnected;
        public boolean m_turnCoderConnected;

        private SwerveModulePosition m_currentPosition = new SwerveModulePosition();
        private SwerveModuleState m_currentState = new SwerveModuleState();
        private final SparkPIDController m_drivePIDController;
        private final SparkPIDController m_turnPIDController;

        public SwerveModule(
                        ModulePosition modulePosition,
                        int driveMotorCanChannel,
                        int turnMotorCanChannel,
                        int cancoderCanChannel,
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
                m_turnEncoderOffsetDeg = turnEncoderOffsetDeg;
                m_turnCANcoder = new CANcoder(cancoderCanChannel);
                CANcoderConfiguration canConfig = new CANcoderConfiguration();
                canConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
                canConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
                m_turnCANcoder.getConfigurator().apply(canConfig);

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
                // direction of the steering motor in the Module.
                m_turnSparkMax.setInverted(turnMotorInverted);

                // Set the PID gains for the driving motor
                m_drivePIDController.setP(ModuleConstants.kDriveP);
                m_drivePIDController.setI(ModuleConstants.kDriveI);
                m_drivePIDController.setD(ModuleConstants.kDriveD);
                m_drivePIDController.setFF(ModuleConstants.kDriveFF);
                m_drivePIDController.setOutputRange(ModuleConstants.kDriveMinOutput,
                                ModuleConstants.kDriveMaxOutput);

                // Set the PID gains for the turning motor
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

                // Calc Relative Encoder Correction
                m_turnAngleCorrectionRad = Math.abs(getAbsolutePositionRad());

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
                correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_turnAngleCorrectionRad));

                // Optimize the reference state to avoid spinning further than 90 degrees.
                SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
                                new Rotation2d(getAbsolutePositionRad()));

                // Command driving and turning SPARKS MAX towards their respective setpoints.
                m_drivePIDController.setReference(optimizedDesiredState.speedMetersPerSecond,
                                CANSparkMax.ControlType.kVelocity);
                m_turnPIDController.setReference(optimizedDesiredState.angle.getRadians(),
                                CANSparkMax.ControlType.kPosition);

                // Set current state and position
                m_currentState = optimizedDesiredState;
                m_currentPosition = new SwerveModulePosition(
                                m_currentPosition.distanceMeters + (m_currentState.speedMetersPerSecond * 0.02),
                                m_currentState.angle);
        }

        public SwerveModulePosition getPosition() {
                return m_currentPosition;
        }

        public SwerveModuleState getState() {
                return m_currentState;
        }

        private boolean checkCAN() {
                m_driveMotorConnected = m_driveSparkMax.getFirmwareVersion() != 0;
                m_turnMotorConnected = m_turnSparkMax.getFirmwareVersion() != 0;
                m_turnCoderConnected = m_turnCANcoder.getDeviceID() != 0;

                return m_driveMotorConnected && m_turnMotorConnected && m_turnCoderConnected;
        }

        public double getAbsolutePositionRad() {
                double absPosRad = Units.rotationsToRadians(m_turnCANcoder.getAbsolutePosition().getValueAsDouble());
                absPosRad -= Units.degreesToRadians(m_turnEncoderOffsetDeg);
                return absPosRad;
        }

        public double getTurnPositionRad() {
                double conv = 0;
                double fullPosDeg = Units.radiansToDegrees(m_turnEncoder.getPosition() + m_turnAngleCorrectionRad);

                if (fullPosDeg < 0)
                        conv = 360;

                double posDeg = conv + fullPosDeg % 360;
                return Units.degreesToRadians(posDeg);
        }
}
