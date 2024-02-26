package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.intake.IntakePivot;

public class WaitIntakeInPosition extends Command {
    private final IntakePivot.IntakePosition position;

    public WaitIntakeInPosition(IntakePivot.IntakePosition position) {
        this.position = position;
    }

    @Override
    public boolean isFinished() {
        return Subsystems.intake.isInPosition(position);
    }
}
