package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.InitializeAutoState;

public class DebugAuto extends SequentialCommandGroup {
  public DebugAuto() {
    // double speed = 0.5;
    // double offset = -90.0;
    addCommands(
      new InitializeAutoState(0),
      new PrintCommand("DEBUG AUTO")
      // Subsystems.swerveSubsystem.getAutoPath("Test")
      // new RotateToAngle(0)
      //   .withTimeout(3)
    );
    //   new SchedulePose(Pose.Stow),
    //   new ProfiledDistanceDriveCommand(0, 0.5, 3, 0)
    //     .withEndSpeed(0),
    //     new ProfiledDistanceDriveCommand(0, 0, 0, 0)
    // );
  }
}
