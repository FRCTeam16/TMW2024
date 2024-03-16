package frc.robot.subsystems.pose;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.util.BSLogger;
import frc.robot.subsystems.util.Counter;

public class FeedNoteToShooterCommandVelocity extends Command {
    public FeedNoteToShooterCommandVelocity() {
        addRequirements(Subsystems.shooter);
    }
    @Override
    public void initialize() {
        BSLogger.log("FeedNoteToShooterCommand", "starting");
        if (!Subsystems.shooter.isNoteDetected()) {
            Subsystems.shooter.getFeeder().setSlowFeedMode(false);
            Subsystems.intake.setIntakeState(Intake.IntakeState.FeedNote);
            Subsystems.shooter.getFeeder().setVelocityMode(true);
            Subsystems.asyncManager.register("FeedNoteToShooterCommand", this::handleFeedingNote);
        }
    }

    @Override
    public boolean isFinished() {
        return Subsystems.shooter.isNoteDetected();
    }

    void handleFeedingNote() {
        Subsystems.shooter.getFeeder().receiveFromIntakeVelocity();
        Subsystems.shooter.getFeeder().applyVelocity();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            BSLogger.log("FeedNoteToShooterCommand", "STOPPED DUE TO INTERRUPT");
        }
        Subsystems.shooter.getFeeder().setOpenLoopSetpoint(0.0);
        Subsystems.shooter.getFeeder().setVelocityMode(false);
        Subsystems.asyncManager.unregister("FeedNoteToShooterCommand");
    }
}
