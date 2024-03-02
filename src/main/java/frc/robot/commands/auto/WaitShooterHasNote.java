package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.util.BSLogger;

public class WaitShooterHasNote extends Command {

    @Override
    public void initialize() {
        BSLogger.log("WaitShooterHasNote", "starting");
    }

    @Override
    public boolean isFinished() {
        if (Subsystems.shooter.isNoteDetected()) {
            BSLogger.log("WaitShooterHasNote", "detected note");
            return true;
        }
        return false;
    }
}
