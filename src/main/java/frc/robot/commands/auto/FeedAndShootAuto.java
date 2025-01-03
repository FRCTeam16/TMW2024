package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.auto.strategies.CommonCommands;
import frc.robot.subsystems.VisionAimManager;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.pose.PoseManager;
import frc.robot.subsystems.util.BSLogger;

public class FeedAndShootAuto extends SequentialCommandGroup {
    public FeedAndShootAuto(VisionAimManager.ShootingProfile profile) {
        addCommands(
                Commands.parallel(
                        Commands.runOnce(() -> BSLogger.log("FeedAndShootAuto", "starting")),
                        new WaitIntakeHasNoteCommand(),
                        Commands.runOnce(() -> Subsystems.pivot.applyShootingProfile(profile), Subsystems.pivot),
                        Commands.runOnce(() -> Subsystems.shooter.applyShootingProfile(profile), Subsystems.shooter)
                ).withTimeout(1.0),
                Commands.parallel(
                        Commands.runOnce(() -> BSLogger.log("FeedAndShootAuto", "waiting for position and speed")),
                        new WaitPivotInPosition(),
                        new WaitShooterAtSpeed(),
                        new WaitIntakeInPosition(IntakePivot.IntakePosition.Zero)
                ).withTimeout(1.0),
                Commands.parallel(
                        Commands.runOnce(() -> BSLogger.log("FeedAndShootAuto", "feed and shoot")),
                        Commands.runOnce(() -> Subsystems.shooter.getFeeder().startFeed(), Subsystems.shooter),
                        Subsystems.intake.moveToStateCmd(Intake.IntakeState.FeedNote),
                        new WaitCommand(0.15), // time to feed from intake to shooter
                        Commands.runOnce(() -> Subsystems.intake.getIntakePivot().setIntakePosition(IntakePivot.IntakePosition.Pickup)) //hack
                ),
                new WaitCommand(0.3),
                Commands.parallel(
                        Commands.runOnce(() -> BSLogger.log("FeedAndShootAuto", "finished")),
                        Commands.runOnce(Subsystems.shooter::stopFeeder, Subsystems.shooter),
                        Subsystems.poseManager.getPoseCommand(PoseManager.Pose.PickupNoTrap)
                ));
    }

    public static Command createStandardShot(VisionAimManager.ShootingProfile profile) {
        return Commands.sequence(
                Commands.parallel(
                        Commands.runOnce(() -> BSLogger.log("FeedAndShootAuto | std", "starting - wait for intake has note")),
                        new WaitIntakeHasNoteCommand(),
                        Commands.runOnce(() -> Subsystems.shooter.applyShootingProfile(profile), Subsystems.shooter)
                ).withTimeout(1.0),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.FeedNoteToShooter).withTimeout(1.0), 
                Commands.parallel(
                        Commands.runOnce(() -> BSLogger.log("FeedAndShootAuto | std", "Running standard shot")),
                        CommonCommands.DoShotCommand(profile),
                        new WaitCommand(0.25)
                ),
                // Subsystems.poseManager.getPoseCommand(PoseManager.Pose.PickupNoTrap),
                Commands.runOnce(() -> BSLogger.log("FeedAndShootAuto | std", "finished"))
        );
    }
}
