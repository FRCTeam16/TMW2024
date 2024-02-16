package frc.robot.subsystems.pose;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.intake.Intake;

class PoseCommands {

     static Command moveToStartingConfigPose() {
        return new SequentialCommandGroup(
                Commands.parallel(
                        Subsystems.intake.moveToStateCmd(Intake.IntakeState.StartingPosition),
                        Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.StartingPosition)
                ));
    }

    static Command moveToPickupPose() {
        return new SequentialCommandGroup(
                Commands.parallel(
                        Subsystems.intake.moveToStateCmd(Intake.IntakeState.IntakeFromFloor)
                ));
    }

     static Command moveToDrivePose() {
        return new SequentialCommandGroup(
                Commands.parallel(
                        Subsystems.intake.moveToStateCmd(Intake.IntakeState.HoldNote),
                        Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.FeedPosition)
                ));
    }

    static Command moveToFeedShooterPose() {
         return Commands.none();
    }

    static Command moveToFeedTrapPose() {
         return Commands.none();
    }

    static Command moveToClimbPose() {
        return Commands.none();
    }

}
