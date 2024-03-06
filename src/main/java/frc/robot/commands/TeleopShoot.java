package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.pose.PoseManager;
import frc.robot.subsystems.trap.Trap;
import frc.robot.subsystems.util.BSLogger;

public class TeleopShoot extends Command {

    @Override
    public void initialize() {
        boolean noteDetected = Subsystems.intake.isNoteDetected();
//        boolean inPose = Subsystems.trap.getTrapState() == Trap.TrapState.AmpShotExtend;
//        boolean inPose = Subsystems.poseManager.getLastPose() == PoseManager.Pose.PositionForAmp;
        boolean inPose = Subsystems.trap.getExtender().isExtended();

        BSLogger.log("TeleopShoot", "Note? %b | Pose? %b".formatted(noteDetected, inPose));

        if (!noteDetected || !inPose) {
            BSLogger.log("TeleopShoot", "Running shooter");
            Subsystems.shooter.runShooter();
            Subsystems.shooter.shoot();
        } else {
            BSLogger.log("TeleopShoot", "Running FireAmp");
            Subsystems.poseManager.getPoseCommand(PoseManager.Pose.FireAmpShot).asProxy().schedule();
        }
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}