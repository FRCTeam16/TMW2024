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
                Commands.runOnce(Subsystems.shooter::stopFeeder)
        );
    }

    static Command feedNoteToShooterPose() {
        return new SequentialCommandGroup(
                // TODO: Need to update movement commands to have true end state positions
                Commands.parallel(
                        Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.FeedPosition),
                        Subsystems.intake.moveToStateCmd(Intake.IntakeState.HoldNote)
                ),
                new FeedNoteToShooterCommand().withTimeout(2.0),
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
        return new ConditionalCommand(
                new ParallelCommandGroup(
                        Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.FeedPosition),
                        Subsystems.intake.moveToStateCmd(Intake.IntakeState.HoldNote)
                ),
                new PrintCommand("Ignoring drive request since we in transition with note"),
                () -> !(Intake.IntakeState.RotateUpWhileFeedingNote.equals(Subsystems.intake.getIntakeState())));

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

    public static Command shooterAimVisionPose() {
        return new SequentialCommandGroup(
                Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.VisionAim)
        );
    }
}
