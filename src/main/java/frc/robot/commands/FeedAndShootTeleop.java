package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.auto.strategies.CommonCommands;
import frc.robot.commands.auto.WaitIntakeHasNoteCommand;
import frc.robot.commands.auto.WaitIntakeInPosition;
import frc.robot.commands.auto.WaitPivotInPosition;
import frc.robot.commands.auto.WaitShooterAtSpeed;
import frc.robot.subsystems.VisionAimManager;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.pose.PoseManager;
import frc.robot.subsystems.util.BSLogger;

public class FeedAndShootTeleop extends SequentialCommandGroup {
    public FeedAndShootTeleop(VisionAimManager.ShootingProfile profile) {
        addCommands(
                Commands.parallel(
                        Commands.runOnce(() -> BSLogger.log("FeedAndShootTeleop", "starting")),
                        new WaitIntakeHasNoteCommand(),
                        Commands.runOnce(() -> Subsystems.pivot.applyShootingProfile(profile), Subsystems.pivot),
                        Commands.runOnce(() -> Subsystems.shooter.applyShootingProfile(profile), Subsystems.shooter)
                ).withTimeout(1.0),
                Commands.parallel(
                        Commands.runOnce(() -> BSLogger.log("FeedAndShootTeleop", "waiting for position and speed")),
                        new WaitPivotInPosition(),
                        new WaitShooterAtSpeed(),
                        new WaitIntakeInPosition(IntakePivot.IntakePosition.Zero)
                ).withTimeout(1.0),
                Commands.parallel(
                        Commands.runOnce(() -> BSLogger.log("FeedAndShootTeleop", "feed and shoot")),
                        Commands.runOnce(() -> Subsystems.shooter.getFeeder().startFeed(), Subsystems.shooter),
                        Subsystems.intake.moveToStateCmd(Intake.IntakeState.FeedNote),
                        new WaitCommand(0.15) // time to feed from intake to shooter
//                        Commands.runOnce(() -> Subsystems.intake.getIntakePivot().setIntakePosition(IntakePivot.IntakePosition.Pickup)) //hack
                ),
                new WaitCommand(0.3),
                Commands.parallel(
                        Commands.runOnce(() -> BSLogger.log("FeedAndShootTeleop", "finished")),
                        Commands.runOnce(Subsystems.shooter::stopFeeder, Subsystems.shooter),
                        Commands.runOnce(() -> Subsystems.intake.moveToStateCmd(Intake.IntakeState.StopFeed))
//                        Subsystems.poseManager.getPoseCommand(PoseManager.Pose.PickupNoTrap)
                ));
    }
}
