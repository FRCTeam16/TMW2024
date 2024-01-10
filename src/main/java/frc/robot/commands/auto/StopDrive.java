package frc.robot.commands.auto;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;

public class StopDrive extends Command {
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    public StopDrive() {
        addRequirements(Subsystems.swerveSubsystem);
    }

    @Override
    public void execute() {
        System.out.println("STOP DRIVE");
        Subsystems.swerveSubsystem.applyRequest(() -> brake);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
