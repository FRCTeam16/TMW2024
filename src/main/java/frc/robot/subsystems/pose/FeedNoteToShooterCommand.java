package frc.robot.subsystems.pose;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.FeederHelper;
import frc.robot.subsystems.util.BSLogger;
import frc.robot.subsystems.util.Counter;

public class FeedNoteToShooterCommand extends Command {
    public FeedNoteToShooterCommand() {
//        addRequirements(Subsystems.shooter);
    }
    @Override
    public void initialize() {
        BSLogger.log("FeedNoteToShooterCommand", "starting");
        if (!Subsystems.shooter.isNoteDetected()) {
            Subsystems.shooter.getFeeder().setSlowFeedMode(false);
            Subsystems.intake.setIntakeState(Intake.IntakeState.FeedNote);
            Subsystems.asyncManager.register("FeedNoteToShooterCommand", this::handleFeedingNote);
        }
    }

    @Override
    public boolean isFinished() {
        return Subsystems.shooter.isNoteDetected();
    }

    void handleFeedingNote() {
        Subsystems.shooter.getFeeder().receiveFromIntake();
        Subsystems.shooter.getFeeder().applyOpenLoop();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            BSLogger.log("FeedNoteToShooterCommand", "STOPPED DUE TO INTERRUPT");
        }
        Subsystems.asyncManager.unregister("FeedNoteToShooterCommand");
    }
}
