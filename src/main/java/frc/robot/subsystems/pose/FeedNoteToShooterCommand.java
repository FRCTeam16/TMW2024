package frc.robot.subsystems.pose;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.util.BSLogger;
import frc.robot.subsystems.util.Counter;

public class FeedNoteToShooterCommand extends Command {
    Counter holdCount = new Counter().withThreshold(2);

    @Override
    public void initialize() {
        BSLogger.log("FeedNoteToShooterCommand", "starting");
        Subsystems.shooter.feedNote();
        Subsystems.intake.setIntakeState(Intake.IntakeState.FeedNote);
    }

    @Override
    public boolean isFinished() {
        if (Subsystems.shooter.isNoteDetected()) {
            if (holdCount.increment()) {
                Subsystems.shooter.stopFeeder();
                BSLogger.log("FeedNoteToShooterCommand", "finished");
                return true;
            }
        } else {
            holdCount.reset();
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            BSLogger.log("FeedNoteToShooterCommand", "STOPPED DUE TO INTERRUPT");
        }
    }
}
