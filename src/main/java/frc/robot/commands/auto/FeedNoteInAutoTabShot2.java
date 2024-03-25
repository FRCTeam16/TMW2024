package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems;
import frc.robot.auto.strategies.Tab;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.pose.PoseManager;
import frc.robot.subsystems.util.BSLogger;

/**
 * A command that feeds a note into the robot in autonomous mode.
 */
public class FeedNoteInAutoTabShot2 extends SequentialCommandGroup {
    public FeedNoteInAutoTabShot2() {
        addCommands(
                Commands.parallel(
                        Commands.runOnce(() -> BSLogger.log("FeedNoteInAutoTabShot2", "starting")),
                        new WaitIntakeHasNoteCommand(),
                        new EnableShooterCommand(),
                        Commands.runOnce(() -> Subsystems.pivot.applyShootingProfile(Tab.SecondShot))
                ).withTimeout(1.0),
                Commands.parallel(
                        Commands.runOnce(() -> Subsystems.pivot.applyShootingProfile(Tab.SecondShot)),
                        Commands.runOnce(() -> Subsystems.shooter.applyShootingProfile(Tab.SecondShot)),
                        new WaitIntakeInPosition(IntakePivot.IntakePosition.Zero)
                ).withTimeout(1.0),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.FeedNoteShooterCustomPivot),
                Commands.runOnce(() -> BSLogger.log("FeedNoteInAutoTabShot2", "finished")));
    }
}
