// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Map;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.constants.ModuleConstants;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;

/** Add your docs here. */
public class ShuffleboardContent {

        static ShuffleboardLayout boolsLayout;

        public ShuffleboardContent() {

        }

        public static void initBooleanShuffleboard(SwerveModule sm) {
                int moduleNumber = sm.m_modulePosition.ordinal();
                String abrev = ModuleConstants.modAbrev[moduleNumber];

                ShuffleboardTab x = Shuffleboard.getTab("Drivetrain");

                x.addBoolean("DriveCAN" + abrev, () -> sm.m_driveMotorConnected)
                                .withPosition(8, moduleNumber);
                x.addBoolean("TurnCAN" + abrev, () -> sm.m_turnMotorConnected)
                                .withPosition(9, moduleNumber);

        }

        public static void initDriveShuffleboard(SwerveModule sm) {
                int moduleNumber = sm.m_modulePosition.ordinal();
                String abrev = ModuleConstants.modAbrev[moduleNumber];
                String driveLayout = sm.m_modulePosition.toString() + " Drive";

                ShuffleboardLayout drLayout = Shuffleboard.getTab("Drivetrain")
                                .getLayout(driveLayout, BuiltInLayouts.kList).withPosition(moduleNumber * 2, 0)
                                .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));

                drLayout.addNumber("Drive Speed MPS " + abrev, () -> sm.m_driveEncoder.getVelocity());
                drLayout.addNumber("Drive Position DEG" + abrev, () -> sm.m_driveEncoder.getPosition());
                drLayout.addNumber("App Output " + abrev, () -> sm.m_driveMotor.getAppliedOutput());
                drLayout.addNumber("Current Amps " + abrev, () -> sm.m_driveMotor.getOutputCurrent());
                drLayout.addNumber("Firmware" + abrev, () -> sm.m_driveMotor.getFirmwareVersion());
        }

        public static void initTurnShuffleboard(SwerveModule sm) {
                int moduleNumber = sm.m_modulePosition.ordinal();
                String abrev = ModuleConstants.modAbrev[moduleNumber];
                String turnLayout = sm.m_modulePosition.toString() + " Turn";

                ShuffleboardLayout tuLayout = Shuffleboard.getTab("Drivetrain")
                                .getLayout(turnLayout, BuiltInLayouts.kList).withPosition(moduleNumber * 2, 2)
                                .withSize(2, 3).withProperties(Map.of("Label position", "LEFT"));

                if (sm.m_state != null) {
                        tuLayout.addNumber("Turn Setpoint Deg " + abrev, () -> sm.m_state.angle.getDegrees());
                }
                tuLayout.addNumber("Turn Enc Pos " + abrev, () -> sm.m_turnEncoder.getPosition() % 360);
                tuLayout.addNumber("Act Ang Deg " + abrev, () -> sm.m_turnEncoder.getPosition());
                tuLayout.addNumber("TurnAngleOut" + abrev, () -> sm.m_turnMotor.getAppliedOutput());
                tuLayout.addNumber("Abs Position" + abrev, () -> sm.m_turnCANcoder.getPosition().getValueAsDouble());
                tuLayout.addNumber("Current Amps" + abrev, () -> sm.m_turnMotor.getOutputCurrent());
                tuLayout.addNumber("Abs Offset" + abrev, () -> sm.m_turnEncoderOffset);
                tuLayout.addNumber("Firmware" + abrev, () -> sm.m_turnMotor.getFirmwareVersion());
        }

        public static void initCoderBooleanShuffleboard(SwerveModule sm) {
                int moduleNumber = sm.m_modulePosition.ordinal();
                String abrev = ModuleConstants.modAbrev[moduleNumber];
                ShuffleboardTab x = Shuffleboard.getTab("CanCoders");

                x.addBoolean("CANCoder Connected" + abrev, () -> sm.m_turnCoderConnected)
                                .withPosition(8, moduleNumber * 2);

                StatusSignal<Integer> faults = sm.m_turnCANcoder.getFaultField();
                x.addBoolean("CANCoder Faults" + abrev, () -> 0 != faults.getValue())
                                .withPosition(9, moduleNumber * 2);

        }

        public static void initCANCoderShuffleboard(SwerveModule sm) {
                int moduleNumber = sm.m_modulePosition.ordinal();
                String abrev = ModuleConstants.modAbrev[moduleNumber];
                String canCoderLayout = sm.m_modulePosition.toString() + " CanCoder";

                ShuffleboardLayout coderLayout = Shuffleboard.getTab("CanCoders")
                                .getLayout(canCoderLayout, BuiltInLayouts.kList).withPosition(moduleNumber * 2, 0)
                                .withSize(2, 3).withProperties(Map.of("Label position", "LEFT"));

                coderLayout.addNumber("Abs Position" + abrev,
                                () -> sm.m_turnCANcoder.getAbsolutePosition().getValueAsDouble());
                coderLayout.addNumber("Abs Offset" + abrev, () -> sm.m_turnEncoderOffset);
                coderLayout.addNumber("Position" + abrev, () -> sm.m_turnCANcoder.getPosition().getValueAsDouble());
                coderLayout.addNumber("Velocity" + abrev, () -> sm.m_turnCANcoder.getVelocity().getValueAsDouble());
        }

        public static void initGyro(SwerveDrive sd) {
                ShuffleboardTab drLayout1 = Shuffleboard.getTab("Gyro");

                drLayout1.addNumber("Gyroscope Angle", () -> sd.getYaw()).withPosition(7, 4)
                                .withSize(1, 1);
                drLayout1.addNumber("Yaw", () -> sd.getYaw()).withPosition(8, 4)
                                .withSize(1, 1);
                drLayout1.addNumber("Roll", () -> sd.getRoll()).withPosition(9, 4)
                                .withSize(1, 1);
                drLayout1.addNumber("Pitch", () -> sd.getPitch()).withPosition(9, 4)
                                .withSize(1, 1);
        }
}
