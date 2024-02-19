package frc.robot.subsystems.pose;

import edu.wpi.first.wpilibj2.command.*;
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
        return new ParallelCommandGroup(
                Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.FeedPosition),
                Subsystems.intake.moveToStateCmd(Intake.IntakeState.IntakeFromFloor),
                Commands.runOnce(Subsystems.shooter::stopFeeder)    // TODO: make command factory on Shooter
        );
    }

    static Command feedNoteToShooterPose() {
        return new SequentialCommandGroup(
                // TODO: Need to update movement commands to have true end state positions
                Commands.parallel(
                        Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.FeedPosition),
                        Subsystems.intake.moveToStateCmd(Intake.IntakeState.HoldNote)
                ),
                new FeedNoteToShooterCommand(),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.ReadyToShoot)
        );
    }

    static Command readyToShootPose() {
        return Commands.parallel(
                Commands.runOnce(Subsystems.shooter::stopFeeder),
                Subsystems.intake.moveToStateCmd(Intake.IntakeState.HoldNote)
        );
    }

    static Command moveToDrivePose() {
        return new ParallelCommandGroup(
                Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.FeedPosition),
                Subsystems.intake.moveToStateCmd(Intake.IntakeState.HoldNote)
        );
    }


    public static Command moveToNotePickedUpPose() {
        return new ParallelCommandGroup(
                Subsystems.intake.moveToStateCmd(Intake.IntakeState.HoldNote)
        );
    }

    public static Command positionForAmpPose() {
        return new SequentialCommandGroup(
                Subsystems.intake.moveToStateCmd(Intake.IntakeState.AmpAim)
        );
    }
}
