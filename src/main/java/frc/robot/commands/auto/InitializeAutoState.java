package frc.robot.commands.auto;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.ZeroAndSetOffsetCommand;

public class InitializeAutoState extends ParallelCommandGroup {

  /** Creates a new InitializeAutoState. */
  public InitializeAutoState(Measure<Angle> initialRobotAngle) {

    // OLD
    addCommands(
      new ZeroAndSetOffsetCommand(initialRobotAngle.in(Units.Degrees)),
      new InstantCommand(() -> Subsystems.swerveSubsystem.tareEverything()),
      new WaitCommand(0.10)
    );
  }
}
