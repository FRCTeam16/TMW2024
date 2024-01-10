package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.vision.Limelight.CameraMode;
import frc.robot.subsystems.vision.Limelight.LEDMode;
import frc.robot.subsystems.vision.Pipeline;

/**
 */
public class EnableImageProcessing extends Command {

    private Pipeline pipeline;

    public EnableImageProcessing(Pipeline pipeline) { 
        this.pipeline = pipeline;
    }

    public void initialize() {
        Subsystems.visionSubsystem.enable();
        Subsystems.visionSubsystem.getLimelight().setCameraMode(CameraMode.ImageProcessing);
        Subsystems.visionSubsystem.selectPipelineSync(pipeline);
        Subsystems.visionSubsystem.getLimelight().setLEDMode(LEDMode.CurrentPipeline);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
