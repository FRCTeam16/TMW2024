package frc.robot.auto.strategies;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.auto.InitializeAutoState;

/**
 * Strategy to simply run named autos from PathPlanner
 */
public class DebugPathAuto extends AutoPathStrategy {
    public DebugPathAuto(String pathName) {
        addCommands(
            new InitializeAutoState(0),
            new PrintCommand("DebugPathAuto: " + pathName),
            this.runAutoPath(pathName)
        );
    }
}
