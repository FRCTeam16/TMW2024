package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;

public class RunDMSCommand extends Command {
  /** Creates a new RunDMSCommand. */
  public RunDMSCommand() {
    addRequirements(Subsystems.ledSubsystem, Subsystems.swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Subsystems.ledSubsystem.startDMS();
  }

  @Override
  public void end(boolean interrupted) {
    Subsystems.ledSubsystem.stopDMS();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Subsystems.ledSubsystem.isStopped() || DriverStation.isDisabled();
  }
}
