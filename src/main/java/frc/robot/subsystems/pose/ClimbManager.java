package frc.robot.subsystems.pose;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.util.BSLogger;

import java.util.List;


public class ClimbManager {

    private final List<ClimbPose> climbPoses;
    private ClimbPose lastPose = null;
    private int poseIndex = 0;

    public ClimbManager() {
        climbPoses = List.of(ClimbPose.StartClimb, ClimbPose.RotateTrapArmsUp, ClimbPose.PullUp, ClimbPose.PutNoteInTrap, ClimbPose.RetractTrapArms);
    }

    public ClimbPose getLastPose() {
        return lastPose;
    }

    public Command getNextPoseCommand() {
        BSLogger.log("ClimbManager", "getNextPose: poseIndex=" + poseIndex);
        if (poseIndex < climbPoses.size()) {
            lastPose = climbPoses.get(poseIndex);
            poseIndex++;
            return getPoseCommand(lastPose);
        }
        return Commands.print("CLIMB FINISHED");
    }

    public Command getPoseCommand(ClimbPose pose) {
        BSLogger.log("ClimbManager", "getPoseCommand for : " + pose);
        return switch (pose) {
            case StartClimb -> ClimbPoseCommands.startClimbPose();
            case RotateTrapArmsUp -> ClimbPoseCommands.rotateTrapArmsUp();
            case PullUp -> ClimbPoseCommands.pullUp();
            case PutNoteInTrap -> ClimbPoseCommands.putNoteInTrap();
            case RetractTrapArms -> ClimbPoseCommands.clearTrapArms();
        };
    }

    public enum ClimbPose {
        StartClimb,
        RotateTrapArmsUp,
        PullUp,
        PutNoteInTrap,
        RetractTrapArms
    }

}
