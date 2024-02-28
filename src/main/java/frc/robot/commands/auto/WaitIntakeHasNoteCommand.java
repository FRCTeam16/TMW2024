package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.util.BSLogger;

public class WaitIntakeHasNoteCommand extends Command {

    @Override
    public void initialize() {
        BSLogger.log("WaitIntakeHasNoteCommand", "starting");
    }

    @Override
    public boolean isFinished() {
        if (Subsystems.intake.isNoteDetected()) {
            BSLogger.log("WaitIntakeHasNoteCommand", "finished");
            return true;
        }
        return false;
    }
}
