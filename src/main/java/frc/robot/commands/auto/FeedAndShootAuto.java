package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.subsystems.VisionAimManager;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakePivot;
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
                        Commands.runOnce(() -> Subsystems.shooter.getFeeder().startFeed(), Subsystems.shooter)
                ),
                Subsystems.intake.moveToStateCmd(Intake.IntakeState.FeedNote),
                new WaitCommand(0.5),
                Commands.parallel(
                        Commands.runOnce(() -> BSLogger.log("FeedAndShootAuto", "finished"))
                ));
    }
}
