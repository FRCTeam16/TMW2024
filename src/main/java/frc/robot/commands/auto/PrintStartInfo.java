package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.util.BSLogger;

public class PrintStartInfo extends Command {
    private final String message;

    public PrintStartInfo(String message) {
        this.message = message;
    }

    @Override
    public void initialize() {
        BSLogger.log("PrintStartInfo", "[AUTO] Starting: " + message);
        BSLogger.log("PrintStartInfo", "[AUTO] Started at:" + Timer.getFPGATimestamp());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
