package frc.robot.subsystems.pose;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.util.Counter;

public class FeedNoteToShooterCommand extends Command {
    Counter holdCount = new Counter().withThreshold(10);

    @Override
    public void initialize() {
        Subsystems.shooter.feedNote();
        Subsystems.intake.setIntakeState(Intake.IntakeState.FeedNote);
    }

    @Override
    public void execute() {
        if (Subsystems.shooter.isNoteDetected()) {
            holdCount.increment();
        }
    }

    @Override
    public boolean isFinished() {
        return holdCount.isThresholdMet();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            DataLogManager.log("[FeedNoteToShooterCommand] STOPPED DUE TO INTERRUPT");
        }
    }
}
