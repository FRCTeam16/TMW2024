package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.util.BSLogger;

public class EnableShooterCommand extends Command {
    private final boolean enableShooter;

    public EnableShooterCommand() {
        this(true);
        this.addRequirements(Subsystems.shooter, Subsystems.pivot);
    }

    public EnableShooterCommand(boolean enable) {
        this.enableShooter = enable;
    }
    @Override
    public void initialize() {
        BSLogger.log("EnableShooterCommand", String.valueOf(enableShooter));
        if (enableShooter) {
            Subsystems.shooter.runShooter();
        } else {
            Subsystems.shooter.stopShooter();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
