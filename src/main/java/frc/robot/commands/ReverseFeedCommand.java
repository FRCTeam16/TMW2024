package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Subsystems;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.util.BSLogger;

public class ReverseFeedCommand extends Command {

    public ReverseFeedCommand() {
        addRequirements(Subsystems.shooter, Subsystems.intake);
    }

    @Override
    public void initialize() {
        BSLogger.log("ReverseFeedCommand", "starting");
        Subsystems.shooter.reverseFeed();
        Subsystems.intake.setIntakeState(IntakeState.ReverseFeed);
    }

    @Override
    public boolean isFinished() {
        BSLogger.log("ReverseFeedCommand", "finished");
        return Subsystems.intake.isNoteDetected();
    }
    
}
