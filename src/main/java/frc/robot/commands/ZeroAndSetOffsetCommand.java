package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.util.BSLogger;

/**
 * @FIXME Needs testing
 */
public class ZeroAndSetOffsetCommand extends Command {
  private final double offsetAngleDegrees;

  /** Creates a new ZeroAndSetOffsetCommand. */
  public ZeroAndSetOffsetCommand(double offsetAngleDegrees) {
    this.offsetAngleDegrees = offsetAngleDegrees;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    BSLogger.log("ZeroAndSetOffsetCommand", "Initialize (" + offsetAngleDegrees + ") called");
    Subsystems.swerveSubsystem.getPigeon2().setYaw(offsetAngleDegrees);
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
