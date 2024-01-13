package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

public class TeleopSwerveCommand extends Command {

    private final SwerveDrive m_swerveDrive;
    private final DoubleSupplier m_throttleInput;
    private final DoubleSupplier m_strafeInput;
    private final DoubleSupplier m_rotationInput;

    /** Driver control */
    public TeleopSwerveCommand(
        SwerveDrive swerveDriveSubsystem,
            DoubleSupplier throttleInput,
            DoubleSupplier strafeInput,
            DoubleSupplier rotationInput) {
        m_swerveDrive = swerveDriveSubsystem;
        m_throttleInput = throttleInput;
        m_strafeInput = strafeInput;
        m_rotationInput = rotationInput;

        addRequirements(swerveDriveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double throttle = m_throttleInput.getAsDouble();
        double strafe = m_strafeInput.getAsDouble();
        double rotation = m_rotationInput.getAsDouble();

        // square values while keeping original sign
        throttle = Math.signum(throttle) * Math.pow(throttle, 2);
        strafe = Math.signum(strafe) * Math.pow(strafe, 2);
        rotation = Math.signum(rotation) * Math.pow(rotation, 2);

        m_swerveDrive.drive(throttle, strafe, rotation);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
