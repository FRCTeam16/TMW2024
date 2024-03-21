package frc.robot.subsystems.pose;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.util.BSLogger;

public class FeedNoteToShooterCommandVelocity extends Command {
    private boolean intakeFeedStarted = false;

    public FeedNoteToShooterCommandVelocity() {
        addRequirements(Subsystems.shooter, Subsystems.intake);
    }
    @Override
    public void initialize() {
        BSLogger.log("FeedNoteToShooterCommand", "starting");
        intakeFeedStarted = false;
        if (!Subsystems.shooter.isNoteDetected()) {
            Subsystems.shooter.getFeeder().startFeed();
            Subsystems.asyncManager.register("FeedNoteToShooterCommand", this::handleFeedingNote);
            checkAndRunIntake();
        }
    }

    private void checkAndRunIntake() {
        // BSLogger.log("FeedNoteToShooterCommand", "intakeStart? " + intakeFeedStarted + " | Shooter spped? " + Subsystems.shooter.isFeederAtSpeed());
        if (!intakeFeedStarted && Subsystems.shooter.isFeederAtSpeed()) {
            BSLogger.log("FeedNoteToShooterCommandVelocity", "Checking motor speed: " + Subsystems.shooter.getFeeder().getMotorVelocity());
            intakeFeedStarted = true;
            Subsystems.intake.setIntakeState(Intake.IntakeState.FeedNote);
        }
    }

    @Override
    public boolean isFinished() {
        return Subsystems.shooter.isNoteDetected();
    }

    void handleFeedingNote() {
        checkAndRunIntake();
        Subsystems.shooter.getFeeder().receiveFromIntakeVelocity();
        Subsystems.shooter.getFeeder().applyVelocity();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            BSLogger.log("FeedNoteToShooterCommand", "STOPPED DUE TO INTERRUPT");
        } else {
            BSLogger.log("FeedNoteToShooterCommand", "exited normally");
        }
        Subsystems.shooter.getFeeder().setOpenLoopSetpoint(0.0);
        Subsystems.shooter.getFeeder().setVelocityMode(false);
        Subsystems.shooter.stopFeeder();
        Subsystems.asyncManager.unregister("FeedNoteToShooterCommand");
    }
}
