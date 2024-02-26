package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;

public class WaitIntakeHasNoteCommand extends Command {
    @Override
    public boolean isFinished() {
        return Subsystems.intake.isNoteDetected();
    }
}
