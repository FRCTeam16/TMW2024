package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;

public class EnableDriverCamera extends Command {

    @Override
    public void initialize() {
        Subsystems.visionSubsystem.disable();
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
