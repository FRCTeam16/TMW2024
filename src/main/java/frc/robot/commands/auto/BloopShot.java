package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.util.BSLogger;

public class BloopShot extends Command {
    private Timer timer = new Timer();
    private static final double RUN_TIME = 0.5;

    @Override
    public void initialize() {
        BSLogger.log("BloopShot", "starting");
        timer.start();
        Subsystems.shooter.shoot();
    }

    @Override
    public boolean isFinished() {
        if (timer.hasElapsed(RUN_TIME)) {
            BSLogger.log("BloopShot", "finished");
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Subsystems.shooter.stopFeeder();
        Subsystems.shooter.stopShooter();
    }
}
