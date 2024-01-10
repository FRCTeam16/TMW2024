package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RunWithDisabledInstantCommand extends InstantCommand {
  /** Creates a new RunWithDisabledInstantCommand. */
  public RunWithDisabledInstantCommand(Runnable runnable) {
    super(runnable);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
