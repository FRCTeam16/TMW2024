package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.ZeroAndSetOffsetCommand;

public class InitializeAutoState extends ParallelCommandGroup {

  /** Creates a new InitializeAutoState. */
  public InitializeAutoState(double initialRobotAngleDegrees) {

    // OLD
    addCommands(
      new ZeroAndSetOffsetCommand(initialRobotAngleDegrees),
      new InstantCommand(() -> Subsystems.swerveSubsystem.tareEverything()),
      // new InstantCommand(() -> Subsystems.swerveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(Math.toRadians(initialRobotAngleDegrees))))),
      // new InstantCommand(() -> Subsystems.swerveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d()))),
      new WaitCommand(0.10)
    );
  }
}
