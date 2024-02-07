package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.vision.Limelight;

import java.util.Optional;

public class VisionCommand extends Command {
    private final Optional<String> limelightNameOptional;

    protected VisionCommand() {
        this.limelightNameOptional = Optional.empty();
    }

    protected VisionCommand(String limelightName) {
        this.limelightNameOptional = Optional.ofNullable(limelightName);
    }

    protected Limelight getLimelight() {
        return limelightNameOptional
                .map(name -> Subsystems.visionSubsystem.getLimelightByName(name))
                .orElseGet(() -> Subsystems.visionSubsystem.getDefaultLimelight());
    }
}
