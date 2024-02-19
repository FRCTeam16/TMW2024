package frc.robot.subsystems.pose;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.intake.Intake;

// FIXME: Need to add a timeout
public class FeedNoteToShooterCommand extends Command {

    @Override
    public void initialize() {
        Subsystems.shooter.feedNote();
        Subsystems.intake.setIntakeState(Intake.IntakeState.FeedNote);
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return Subsystems.shooter.isNoteDetected();
    }

    @Override
    public void end(boolean interrupted) {
    }
}
