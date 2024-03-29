package frc.robot;

import frc.robot.subsystems.SwerveDrive;
import frc.robot.utils.AutonManager;
import frc.robot.commands.auto.AutoRoutines;
import frc.robot.commands.swerve.TeleopSwerveCommand;
import frc.robot.constants.DriveConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class RobotContainer {
  public final SwerveDrive swerve = new SwerveDrive(); // Swerve drive system

  private final AutonManager autonManager = new AutonManager();
  private final CommandJoystick driver = new CommandJoystick(1);

  public RobotContainer() {
    addAutonomousChoices();
    autonManager.displayChoices();

    configureButtonBindings();
  }

  public Command getAutonomousCommand() {
    return autonManager.getSelected();
  }

  private void addAutonomousChoices() {
        autonManager.addDefaultOption("Shoot Note and Leave",
        AutoRoutines.ShootNoteAndLeave(swerve));
  }

  private void configureButtonBindings() {

    // Swerve drive commands
    swerve.setDefaultCommand(
        new TeleopSwerveCommand(
            swerve,
            () -> -MathUtil.applyDeadband(driver.getY(), DriveConstants.kXYDeadband),
            () -> -MathUtil.applyDeadband(driver.getX(), DriveConstants.kXYDeadband),
            () -> -MathUtil.applyDeadband(driver.getZ(), DriveConstants.kRotationDeadband)));

    driver.button(1).onTrue(new InstantCommand(swerve::toggleFieldRelative));
    driver.button(8).onTrue(new InstantCommand(swerve::zeroGyro));
   };
}