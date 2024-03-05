package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.util.BSLogger;


public class CenterNoteIntakeCommand extends Command {
    public static final int NUM_PHASES = 6;
    private final Timer timer = new Timer();
    private int phase = 0;

    public CenterNoteIntakeCommand() {
    }

    @Override
    public void initialize() {
        phase = 0;
        timer.restart();
    }

    @Override
    public void execute() {
        // Safety
        if (phase == NUM_PHASES) {
            return;
        }

        // Modulo between states
        if (phase % 2 == 0) {
            BSLogger.log("CenterNoteInIntakeCommand", "eject");
            Subsystems.intake.getIntakeSpeed().runIntakeEject();
        } else {
            BSLogger.log("CenterNoteInIntakeCommand", "intake");
            Subsystems.intake.getIntakeSpeed().runIntakeFast();
        }
        if (phase != (NUM_PHASES - 1)) {
            if (timer.hasElapsed(0.1)) {
                phase++;
                timer.reset();
            }
        } else {
            BSLogger.log("CenterNoteInIntakeCommand", "looking for note");
            if (timer.hasElapsed(1.0) || Subsystems.intake.isNoteDetected()) {
                phase++;
                timer.reset();
            }

        }
    }

    @Override
    public boolean isFinished() {
        boolean finished = phase >= NUM_PHASES;
        if (finished) {
            Subsystems.intake.getIntakeSpeed().stopIntake();
        }
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        Subsystems.intake.getIntakeSpeed().stopIntake();
    }
}
