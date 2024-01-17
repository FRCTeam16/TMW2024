package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems;
import frc.robot.commands.auto.InitializeAutoState;

/**
 * Strategy to simply run named autos from PathPlanner
 */
public class DebugPathAuto extends SequentialCommandGroup {
    public DebugPathAuto(String pathName) {
        addCommands(
            new InitializeAutoState(0),
            new PrintCommand("DebugPathAuto: " + pathName),
            Subsystems.swerveSubsystem.getAutoPath(pathName)
        );
    }
}
