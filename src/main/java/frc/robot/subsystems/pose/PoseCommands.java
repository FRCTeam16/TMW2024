package frc.robot.subsystems.pose;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Subsystems;
import frc.robot.commands.CenterNoteIntakeCommand;
import frc.robot.commands.auto.WaitIntakeInPosition;
import frc.robot.commands.auto.WaitPivotInPosition;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.trap.Trap;

class PoseCommands {

    static Command moveToStartingConfigPose() {
        return new SequentialCommandGroup(
                Subsystems.intake.moveToStateCmd(Intake.IntakeState.StartingPosition),
                new WaitCommand(0.25),
                Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.StartingPosition));
    }

    static Command moveToPickupPose() {
        return Commands.sequence(
                Subsystems.trap.moveToStateCmd(Trap.TrapState.Drive),
                new WaitCommand(0.25),  // add wait for intake pivot to be in position
                Commands.parallel(
                        Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.FeedPosition),
                        Subsystems.intake.moveToStateCmd(Intake.IntakeState.IntakeFromFloor),
                        Commands.runOnce(Subsystems.shooter::stopFeeder)

                )
        );
    }

    static Command feedNoteToShooterPose() {
        return new SequentialCommandGroup(
                Commands.parallel(
                        Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.FeedPosition),
                        Subsystems.intake.moveToStateCmd(Intake.IntakeState.FeedNote)
                ),
                new FeedNoteToShooterCommand().withTimeout(2.0),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.ReadyToShoot)
        );
    }

    static Command readyToShootPose() {
        return Commands.parallel(
                Commands.runOnce(Subsystems.pivot::moveToQueuedProfile),
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
        return Commands.sequence(
                Commands.parallel(
                        Subsystems.trap.moveToStateCmd(Trap.TrapState.AmpShot),
                        Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.FeedPosition)
                ),
                new WaitPivotInPosition().withTimeout(0.2),
                new CenterNoteIntakeCommand(),
                Commands.parallel(
                        Subsystems.intake.moveToStateCmd(Intake.IntakeState.AmpAim)
                        // TODO make sure our trap arms are in the correct position
                ),
                new WaitIntakeInPosition(IntakePivot.IntakePosition.AMPShot).withTimeout(0.2));

    }

    public static Command fireAmpShotPose() {
        return Commands.sequence(
                Subsystems.intake.moveToStateCmd(Intake.IntakeState.TryShootAmp),
                new WaitCommand(1.0),
                Commands.parallel(
                        Subsystems.intake.moveToStateCmd(Intake.IntakeState.HoldNote),
                        Subsystems.trap.moveToStateCmd(Trap.TrapState.Default)
                )
        );
    }

    public static Command shooterAimVisionPose() {
        return Commands.parallel(
                Commands.runOnce(Subsystems.shooter::runShooter),
                Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.VisionAim)
        );
    }

    public static Command shortShotPose() {
        return Commands.parallel(
                Commands.runOnce(Subsystems.shooter::runShooter),
                Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.ShortShot)
        );
    }

    public static Command startClimbPose() {
        return Commands.sequence(
                Commands.parallel(
                        Commands.runOnce(Subsystems.shooter::stopShooter),
                        Subsystems.trap.moveToStateCmd(Trap.TrapState.Climb),
                        Commands.runOnce(Subsystems.shooter::stopFeeder),
                        Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.Up),
                        Subsystems.climber.moveToStateCmd(Climber.ClimberPosition.UP)),
                // TODO : check wait between trap arm and intake
                Subsystems.intake.moveToStateCmd(Intake.IntakeState.Climb));
    }

    public static Command prepareBloopShotPose() {
        return Commands.parallel(
                Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.StartingPosition),
                Commands.runOnce(() -> Subsystems.shooter.applyShootingProfile(Subsystems.shooter.BloopProfile))
        );
    }

}
