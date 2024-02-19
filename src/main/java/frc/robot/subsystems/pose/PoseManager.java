package frc.robot.subsystems.pose;

import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

public class PoseManager {


    public enum Pose {
        StartingConfig,
        Drive,
        Pickup,
        NotePickedUp,
        Handoff,

    }

    private Pose currentPose = Pose.StartingConfig;
    private Pose lastPose = Pose.StartingConfig;
    private final Map<Pose, Supplier<Command>> registry = new HashMap<>();

    private final StringLogEntry poseLog;

    public PoseManager() {
        poseLog = new StringLogEntry(DataLogManager.getLog(), "/pose");

        registry.put(Pose.Pickup, PoseCommands::moveToPickupPose);
        registry.put(Pose.Handoff, PoseCommands::moveToHandoffPose);
        registry.put(Pose.NotePickedUp, PoseCommands::moveToNotePickedUpPose);
        registry.put(Pose.Drive, PoseCommands::moveToDrivePose);
    }

    public Command getPoseCommand(Pose requestedPose) {
        DataLogManager.log("[PoseManager] Requested pose: " + requestedPose);

        if (registry.containsKey(requestedPose)) {
            lastPose = currentPose;
            currentPose = requestedPose;
            poseLog.append(requestedPose.name());
            return registry.get(requestedPose).get();
        } else {
            String message = "[PoseManager] !!! Unhandled pose requested: " + requestedPose;
            DataLogManager.log(message);
            return Commands.none();
        }
    }

    public Pose getCurrentPose() {
        return currentPose;
    }

    public Pose getLastPose() {
        return lastPose;
    }

    /**
     * Schedule a pose to be performed on the command queue
     * @param pose
     */
    public void schedulePose(Pose pose) {
        CommandScheduler.getInstance().schedule(getPoseCommand(pose));
    }

}
