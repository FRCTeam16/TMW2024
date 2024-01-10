package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;

/**
 * @FIXME Needs testing
 */
public class ZeroAndSetOffsetCommand extends Command {
  private double offsetAngleDegrees;

  /** Creates a new ZeroAndSetOffsetCommand. */
  public ZeroAndSetOffsetCommand(double offsetAngleDegrees) {
    this.offsetAngleDegrees = offsetAngleDegrees;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ZeroANdSetOffsetCommand(" + offsetAngleDegrees + ") called");
    Subsystems.swerveSubsystem.getPigeon2().setYaw(offsetAngleDegrees); // TODO: Test offset is correctly handled
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
