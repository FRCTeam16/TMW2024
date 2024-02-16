package frc.robot.subsystems.pose;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.intake.Intake;

public class MoveToStartingConfigPose extends SequentialCommandGroup {
    public MoveToStartingConfigPose() {
        addCommands(
                Commands.parallel(
                        Subsystems.intake.moveToStateCmd(Intake.IntakeState.StartingPosition),
                        Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.StartingPosition)
                )
        );
    }
}
