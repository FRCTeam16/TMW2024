package frc.robot.subsystems.pose;

import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.util.BSLogger;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

public class PoseManager {


    public final ClimbManager climbManager = new ClimbManager();

    public enum Pose {
        // Default starting configuration
        StartingConfig,
        // General purpose pose for driving around
        Drive,
        // Pose for actively picking up notes from the ground
        Pickup,
        // Transitional pose for handling state transitions between pickup and decision point
        NotePickedUp,
        // Transitional pose for feeding the note from the intake into the shooter
        FeedNoteToShooter,
        // Pose for when the robot is loaded and ready to shoot
        ReadyToShoot,
        // Robot pose used in preparation for scoring on the amp
        PositionForAmp,
        // Robot pose used to fire the amp shot
        FireAmpShot,
        // Pose for automatically adjusting shooter pivot based on vision
        ShooterAimVision,
        AutoShortShot,
        // Start climb
        @Deprecated StartClimb,
        TryClearNote,
        @Deprecated ExtendClimbers,
        FireBigShot,
        SubShot,
        FireShootOverSmiley,
        PrepareBloopShot
    }

    private Pose lastPose = Pose.StartingConfig;
    private final Map<Pose, Supplier<Command>> registry = new HashMap<>();

    private final StringLogEntry poseLog;

    public PoseManager() {
        var pc = new PoseCommands();
        poseLog = new StringLogEntry(DataLogManager.getLog(), "datalog/posemanager");

        registry.put(Pose.StartingConfig, PoseCommands::moveToStartingConfigPose);
        registry.put(Pose.Pickup, PoseCommands::moveToPickupPose);
        registry.put(Pose.Drive, PoseCommands::moveToDrivePose);
        registry.put(Pose.NotePickedUp, PoseCommands::moveToNotePickedUpPose);
        registry.put(Pose.FeedNoteToShooter, PoseCommands::feedNoteToShooterPose);
        registry.put(Pose.ReadyToShoot, PoseCommands::readyToShootPose);
        registry.put(Pose.PositionForAmp, PoseCommands::positionForAmpPose);
        registry.put(Pose.FireAmpShot, PoseCommands::fireAmpShotPose);
        registry.put(Pose.ShooterAimVision, PoseCommands::shooterAimVisionPose);
        registry.put(Pose.AutoShortShot, PoseCommands::shortShotPose);
//        registry.put(Pose.StartClimb, PoseCommands::stepClimbPose);
        registry.put(Pose.PrepareBloopShot, PoseCommands::prepareBloopShotPose);
        registry.put(Pose.FireBigShot, PoseCommands::fireBigShot);
        registry.put(Pose.SubShot, PoseCommands::fireSubShot);
        registry.put(Pose.TryClearNote, PoseCommands::tryClearNote);
        registry.put(Pose.FireShootOverSmiley, PoseCommands::fireShootOverSmiley);
    }

    public ClimbManager getClimbManager() {
        return climbManager;
    }

    public Command getPoseCommand(Pose requestedPose) {
        BSLogger.log("PoseManager", "Fetching Pose: " + requestedPose);
        SmartDashboard.putString("PoseManager/RequestedPose", requestedPose.name());

        if (Pose.StartClimb == requestedPose) {
            BSLogger.log("PoseManager", "Special handling for next pose command");
            return Commands.print("Don't use this method to do climbing");
        }

        if (registry.containsKey(requestedPose)) {
            lastPose = requestedPose;
            poseLog.append(requestedPose.name());
            return registry.get(requestedPose).get();
        } else {
            BSLogger.log("PoseManager", "!!! Unhandled pose requested: " + requestedPose);
            return Commands.print("CLIMB IS FINISHED");
        }
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
