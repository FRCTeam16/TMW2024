package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.util.BSLogger;
import frc.robot.subsystems.util.Counter;

public class WaitIntakeInPosition extends Command {
    private final IntakePivot.IntakePosition position;
    private final Counter seenCounter = new Counter().withThreshold(10);

    @Override
    public void initialize() {
        BSLogger.log("WaitIntakeInPosition", "starting");
    }

    public WaitIntakeInPosition(IntakePivot.IntakePosition position) {
        this.position = position;
        addRequirements(Subsystems.intake);
    }

    @Override
    public boolean isFinished() {
        if (Subsystems.intake.isInPosition(position)) {
            seenCounter.increment();
        } else {
            seenCounter.reset();
        }
        if (seenCounter.isThresholdMet()) {
            BSLogger.log("WaitIntakeInPosition", "finished");
        }
        return seenCounter.isThresholdMet();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            BSLogger.log("WaitIntakeInPosition", "ended due to interrupt");
        }
    }
}
