package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.Subsystems;
import frc.robot.subsystems.pose.PoseManager;
import org.junit.jupiter.api.Test;

public class TestSequencingCommands {

    @Test
    public void testOne() {
        RobotContainer robotContainer = new RobotContainer();
        var cmd1 = Subsystems.poseManager.getPoseCommand(PoseManager.Pose.FireAmpShot);
        var cmd2 = Subsystems.poseManager.getPoseCommand(PoseManager.Pose.FireAmpShot);
        assert cmd1 != cmd2;
    }
}
