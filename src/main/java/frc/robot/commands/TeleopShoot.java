package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems;
import frc.robot.subsystems.pose.PoseManager;
import frc.robot.subsystems.util.BSLogger;

public class TeleopShoot extends Command {

    @Override
    public void initialize() {
        boolean noteDetected = Subsystems.intake.isNoteDetected();
        boolean inPose = Subsystems.trap.getExtender().isExtended();

        BSLogger.log("TeleopShoot", "Note? %b | Pose? %b".formatted(noteDetected, inPose));

        final Command fireCommand;
        if (!noteDetected || !inPose) {
            BSLogger.log("TeleopShoot", "Running shooter");
            fireCommand = Subsystems.shooter.shootCmd();

        } else {
            BSLogger.log("TeleopShoot", "Running FireAmp");
            fireCommand = Subsystems.poseManager.getPoseCommand(PoseManager.Pose.FireAmpShot);
        }
        fireCommand.asProxy().schedule();
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}