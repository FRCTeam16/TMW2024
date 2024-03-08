package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

        final double elapsedTime;

        // Modulo between states
//        SmartDashboard.putNumber("Debug/CenterNoteInIntakeCommand/Phase", phase);
        if (phase % 2 == 0) {
            BSLogger.log("CenterNoteInIntakeCommand", "eject");
//            SmartDashboard.putNumber("Debug/CenterNoteInIntakeCommand/Eject", 1);
            Subsystems.intake.getIntakeSpeed().runCenterSpeedEject();
            elapsedTime = 0.08;
        } else {
            BSLogger.log("CenterNoteInIntakeCommand", "intake");
//            SmartDashboard.putNumber("Debug/CenterNoteInIntakeCommand/Eject", 0);
            Subsystems.intake.getIntakeSpeed().runCenterSpeedIntake();
            elapsedTime = 0.1;
        }

        if (timer.hasElapsed(elapsedTime)) {
            phase++;
            timer.reset();
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
