package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.pose.PoseManager;
import frc.robot.subsystems.util.BSLogger;

/**
 * A command that feeds a note into the robot in autonomous mode.
 */
public class FeedNoteInAuto extends SequentialCommandGroup {
    public FeedNoteInAuto() {
        addCommands(
                Commands.parallel(
                        Commands.runOnce(() -> BSLogger.log("FeedNoteInAuto", "starting")),
                        new WaitIntakeHasNoteCommand(),
                        new WaitIntakeInPosition(IntakePivot.IntakePosition.Zero)
                ).withTimeout(2.0),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.FeedNoteToShooter) 
        );
    }
}
