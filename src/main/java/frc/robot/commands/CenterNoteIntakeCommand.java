package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;


public class CenterNoteIntakeCommand extends Command {
    private final Timer timer = new Timer();
    private int phase = 0;
    public CenterNoteIntakeCommand() {}

    @Override
    public void initialize() {
        phase = 0;
        timer.restart();
    }

    @Override
    public void execute() {
        if (phase % 2 == 0) {
            Subsystems.intake.getIntakeSpeed().runIntakeEject();
        } else {
            Subsystems.intake.getIntakeSpeed().runIntakeFast();
        }
        if (timer.hasElapsed(0.1)) {
            phase++;
            timer.reset();
        }
    }

    @Override
    public boolean isFinished() {
        return phase >= 6;
    }

    @Override
    public void end(boolean interrupted) {
        Subsystems.intake.getIntakeSpeed().stopIntake();
    }
}
