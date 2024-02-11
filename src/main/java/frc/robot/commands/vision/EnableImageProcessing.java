package frc.robot.commands.vision;

import frc.robot.subsystems.vision.Pipeline;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.VisionTypes;

/**
 */
public class EnableImageProcessing extends VisionCommand {

    private Pipeline pipeline;

    public EnableImageProcessing(String limelightName, Pipeline pipeline) {
        super(limelightName);
        this.pipeline = pipeline;
    }
    public EnableImageProcessing(Pipeline pipeline) {
        this(null, pipeline);
    }

    public void initialize() {
        Limelight limelight = this.getLimelight();
        limelight.setCameraMode(VisionTypes.CameraMode.ImageProcessing);
        limelight.setPipelineIndex(pipeline.pipelineNumber);
        limelight.setLEDMode(VisionTypes.LEDMode.CurrentPipeline);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
