package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class WaitUntilMatchTimeCommand extends Command {
  private static final double TOTAL_MATCH_TIME = 150;
  private double targetTime;

  public WaitUntilMatchTimeCommand(double time) {
    this.targetTime = TOTAL_MATCH_TIME - time;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getMatchTime() < targetTime;
  }
}
