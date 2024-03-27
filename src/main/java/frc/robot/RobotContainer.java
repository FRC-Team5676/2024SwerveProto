// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.SwerveDrive;
import frc.robot.utils.AutonManager;
import frc.robot.commands.swerve.TeleopSwerveCommand;
import frc.robot.constants.DriveConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.auto.AutoRoutines;


public class RobotContainer {
  // Autonomous manager import
  private final AutonManager m_autonManager = new AutonManager();

  // The driver's controller
  private final CommandJoystick m_driver = new CommandJoystick(1);

  // The robot's subsystems
  private final SwerveDrive m_swerve = new SwerveDrive();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    addAutonomousChoices();
    m_autonManager.displayChoices();

    configureButtonBindings();
  }

  public Command getAutonomousCommand() {
    return m_autonManager.getSelected();
  }

  private void addAutonomousChoices() {
    m_autonManager.addDefaultOption("Shoot Note and Leave",
        AutoRoutines.ShootNoteAndLeave(m_swerve));
  }

  private void configureButtonBindings() {
    m_swerve.setDefaultCommand(
        new TeleopSwerveCommand(
            m_swerve,
            () -> MathUtil.applyDeadband(m_driver.getY(), DriveConstants.kXYDeadband),
            () -> MathUtil.applyDeadband(m_driver.getX(), DriveConstants.kXYDeadband),
            () -> MathUtil.applyDeadband(m_driver.getZ(), DriveConstants.kRotationDeadband)));

    m_driver.button(1).onTrue(new InstantCommand(m_swerve::toggleFieldRelative));
    m_driver.button(8).onTrue(new InstantCommand(m_swerve::zeroGyro));
  }
}
