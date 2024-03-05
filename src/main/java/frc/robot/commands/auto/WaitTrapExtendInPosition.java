package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.util.BSLogger;

public class WaitTrapExtendInPosition extends Command {

    @Override
    public void initialize() {
        BSLogger.log("WaitTrapExtendInPosition", "starting");
    }

    @Override
    public boolean isFinished() {
        return Subsystems.trap.getExtender().isInPosition();
    }
}

