package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems;
import frc.robot.subsystems.pose.PoseManager;
import frc.robot.subsystems.trap.Trap;
import frc.robot.subsystems.util.BSLogger;

public class TeleopShoot extends Command {
    private Command command;

    @Override
    public void initialize() {
        BSLogger.log("TeleopShoot", "CHECKING STATE");
        boolean inPose = Subsystems.trap.getTrapState() == Trap.TrapState.AmpShotExtend;
        BSLogger.log("TeleopShoot", "Subsystems.trap.getTrapState() == Trap.TrapState.AmpShotExtend = " + inPose);

        if (!Subsystems.intake.isNoteDetected()) {
            command = Commands.sequence(
                    Commands.runOnce(Subsystems.shooter::runShooter),
                    Commands.runOnce(Subsystems.shooter::shoot));
        } else if (Subsystems.trap.getTrapState() == Trap.TrapState.AmpShotExtend) {
            command = Subsystems.poseManager.getPoseCommand(PoseManager.Pose.FireAmpShot);
        } else {
            command = Commands.sequence(
                    Commands.runOnce(Subsystems.shooter::runShooter),
                    Commands.runOnce(Subsystems.shooter::shoot));
        }
        command.initialize();
    }

    @Override
    public void execute() {
        command.execute();
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
    }
}
