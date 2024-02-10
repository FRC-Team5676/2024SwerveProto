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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Autonomous manager import
  private final AutonManager autonManager = new AutonManager();

  // The driver's controller
  private final CommandJoystick driver = new CommandJoystick(0);

  // The robot's subsystems
  private final SwerveDrive swerve = new SwerveDrive();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    addAutonomousChoices();
    autonManager.displayChoices();

    configureButtonBindings();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

  private void addAutonomousChoices() {
    /*
     * autonManager.addDefaultOption("Set Cone and Leave",
     * AutoRoutines.PlaceConeAndLeave(lowerArm, upperArm, intakeArm,
     * swerve, xController, yController, zController));
     */
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureButtonBindings() {
    swerve.setDefaultCommand(
        new TeleopSwerveCommand(
            swerve,
            () -> MathUtil.applyDeadband(driver.getY(), DriveConstants.kXYDeadband),
            () -> MathUtil.applyDeadband(driver.getX(), DriveConstants.kXYDeadband),
            () -> MathUtil.applyDeadband(driver.getZ(), DriveConstants.kRotationDeadband)));

    driver.button(1).onTrue(new InstantCommand(swerve::toggleFieldRelative));
    driver.button(2).onTrue(new InstantCommand(swerve::zeroGyro));
  }
}
