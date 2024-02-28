package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.util.Counter;

public class WaitIntakeInPosition extends Command {
    private final IntakePivot.IntakePosition position;
    private final Counter seenCounter = new Counter().withThreshold(5);

    public WaitIntakeInPosition(IntakePivot.IntakePosition position) {
        this.position = position;
    }

    @Override
    public boolean isFinished() {
        if (Subsystems.intake.isInPosition(position)) {
            seenCounter.increment();
        } else {
            seenCounter.reset();
        }
        return seenCounter.isThresholdMet();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            DataLogManager.log("[WaitIntakeInPosition] ended due to interrupt");
        }
    }
}
