package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.util.BSLogger;

public class WaitPivotInPosition extends Command {

    public WaitPivotInPosition() {
        // addRequirements(Subsystems.pivot);
    }

    @Override
    public void initialize() {
        BSLogger.log("WaitPivotInPosition", "starting");
    }

    @Override
    public boolean isFinished() {
        if (Subsystems.pivot.isInPosition()) {
            BSLogger.log("WaitPivotInPosition", "finished");
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            BSLogger.log("WaitPivotInPosition", "interrupted");
        }
    }
}
