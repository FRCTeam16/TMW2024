package frc.robot.subsystems.pose;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Subsystems;
import frc.robot.commands.CenterNoteIntakeCommand;
import frc.robot.commands.auto.WaitIntakeInPosition;
import frc.robot.commands.auto.WaitPivotInPosition;
import frc.robot.commands.auto.WaitTrapExtendInPosition;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.trap.Trap;

class PoseCommands {

    PoseCommands() {
        SmartDashboard.setDefaultNumber("DeflectDelay", 0.25);
    }

    static Command moveToStartingConfigPose() {
        return new SequentialCommandGroup(
                Subsystems.intake.moveToStateCmd(Intake.IntakeState.StartingPosition),
                new WaitCommand(0.25),
                Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.StartingPosition));
    }

    static Command moveToPickupPose() {
        return Commands.sequence(
                Commands.runOnce(() -> Subsystems.intake.getIntakeSpeed().runIntakeFast()),
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
        // Assumes intake is in zero position
        return Commands.sequence(
                Commands.parallel(
                        Subsystems.trap.moveToStateCmd(Trap.TrapState.AmpShotExtend),
                        Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.FeedPosition)
                ),
                new WaitPivotInPosition().withTimeout(0.2),
                new CenterNoteIntakeCommand(),
                Subsystems.intake.moveToStateCmd(Intake.IntakeState.AmpAim),
                Commands.parallel(
                        new WaitTrapExtendInPosition(),
                        new WaitIntakeInPosition(IntakePivot.IntakePosition.AMPShot)
                ).withTimeout(0.5),
                Subsystems.trap.moveToStateCmd(Trap.TrapState.AmpShotPivot)
                        .unless(() -> !Subsystems.trap.getExtender().isInPosition(30)),
                new WaitCommand(0.25));
    }

    public static Command fireAmpShotPose() {
        return Commands.sequence(
                Subsystems.intake.moveToStateCmd(Intake.IntakeState.TryShootAmp),
                new WaitCommand(0.25),
                Subsystems.trap.moveToStateCmd(Trap.TrapState.AmpShotDeflectShot),
                new WaitCommand(1.0),
                Subsystems.trap.moveToStateCmd(Trap.TrapState.PivotDrive),
                new WaitCommand(0.25),
                Subsystems.trap.moveToStateCmd(Trap.TrapState.Default),
                Subsystems.intake.moveToStateCmd(Intake.IntakeState.HoldNote));
    }

    public static Command shooterAimVisionPose() {
        return Commands.sequence(
                Subsystems.trap.moveToStateCmd(Trap.TrapState.Drive),
                new WaitCommand(0.25),  // add wait for intake pivot to be in position
                Commands.parallel(
                        Commands.runOnce(Subsystems.shooter::runShooter),
                        Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.VisionAim)

                )
        );
    }

    public static Command shortShotPose() {
        return Commands.sequence(
                Subsystems.trap.moveToStateCmd(Trap.TrapState.Drive),
                new WaitCommand(0.25),  // add wait for intake pivot to be in position
                Commands.parallel(
                        Commands.runOnce(Subsystems.shooter::runShooter),
                        Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.ShortShot)
                ),
                new WaitPivotInPosition().withTimeout(0.5)
        );
    }

    public static Command stepClimbPose() {
        return Subsystems.poseManager.climbManager.getNextPoseCommand();
    }

    public static Command prepareBloopShotPose() {
        return Commands.sequence(
                Subsystems.trap.moveToStateCmd(Trap.TrapState.Drive),
                new WaitCommand(0.25),  // add wait for intake pivot to be in position
                Commands.parallel(
                        Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.StartingPosition),
                        Commands.runOnce(() -> Subsystems.shooter.applyShootingProfile(Subsystems.shooter.BloopProfile))
                )
        );
    }

    public static Command fireBigShot() {
        return Commands.sequence(
                Commands.runOnce(Subsystems.shooter::runShooter),
                Commands.parallel(
                        Commands.runOnce(() -> Subsystems.shooter.applyShootingProfile(Shooter.BigShotProfile)),
                        Commands.runOnce(() -> Subsystems.pivot.applyShootingProfile(Shooter.BigShotProfile)),
                        Subsystems.trap.moveToStateCmd(Trap.TrapState.Drive)
                ),
                Commands.runOnce(Subsystems.shooter::runShooter),
                new WaitCommand(0.25),
                Commands.runOnce((Subsystems.shooter::shoot)
                ));
    }

    public static Command fireSubShot() {
        return Commands.sequence(
                Commands.runOnce(Subsystems.shooter::runShooter),
                Commands.parallel(
                        Commands.runOnce(() -> Subsystems.shooter.applyShootingProfile(Shooter.SubShotProfile)),
                        Commands.runOnce(() -> Subsystems.pivot.applyShootingProfile(Shooter.SubShotProfile)),
                        Subsystems.trap.moveToStateCmd(Trap.TrapState.Drive)
                ),
                Commands.runOnce(Subsystems.shooter::runShooter),
                new WaitCommand(0.5),
                Commands.runOnce(Subsystems.shooter::shoot));
    }
}
