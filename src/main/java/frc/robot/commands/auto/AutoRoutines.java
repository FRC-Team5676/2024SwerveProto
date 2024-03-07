package frc.robot.commands.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveDrive;

public class AutoRoutines {
        public static Command ShootNoteAndLeave(SwerveDrive swerve) {
                return Commands.sequence(
                                new PathPlannerAuto("Test Auto").withTimeout(5));
        }
}
