package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems;

public class AutoPathStrategy extends SequentialCommandGroup{

    public Command runAutoPath(String pathName) {
        return Subsystems.autoManager.getAutoPath(pathName);
    }
}
