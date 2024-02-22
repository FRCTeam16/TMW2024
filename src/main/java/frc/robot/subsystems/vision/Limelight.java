package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.Optional;
import java.util.function.Consumer;

public class Limelight {
    public static final double DEFAULT_HEIGHT_TO_TARGET = 57.0;
    private final String name;
    private final VisionTypes.LimelightInfo info;

    private final VisionTypes.CameraDistanceValues cameraDistanceValues;

    public Limelight(VisionTypes.LimelightInfo info) {
        this.name = LimelightHelpers.sanitizeName(info.name());
        this.info = info;
        this.setLEDMode(VisionTypes.LEDMode.CurrentPipeline);
        this.setCameraMode(VisionTypes.CameraMode.ImageProcessing);
        this.cameraDistanceValues = new VisionTypes.CameraDistanceValues(
                this.info.heightToCamera(), DEFAULT_HEIGHT_TO_TARGET, this.info.cameraAngle());
    }

    public VisionTypes.LimelightInfo getInfo() {
        return this.info;
    }

    public String getName() {
        return this.name;
    }

    public void setPipelineIndex(int pipelineIndex) {
        LimelightHelpers.setPipelineIndex(this.name, pipelineIndex);
    }

    public double getAprilTagID() {
        return LimelightHelpers.getFiducialID(this.name);
    }

    public double getNeuralClassID() {
        return LimelightHelpers.getNeuralClassID(this.name);
    }

    public Command requestPipeline(int pipelineNumber) {
        return Commands.runOnce(() -> this.setPipelineIndex(pipelineNumber));
    }

    public VisionTypes.TargetInfo getTargetInfo() {
        return new VisionTypes.TargetInfo(
                LimelightHelpers.getTV(this.name),
                LimelightHelpers.getTX(this.name),
                LimelightHelpers.getTY(this.name),
                LimelightHelpers.getTX(this.name),
                this.cameraDistanceValues
        );
    }

    public VisionTypes.PoseInfo getBotPoseForCurrentAlliance() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            Pose2d pose = DriverStation.Alliance.Red.equals(alliance.get()) ?
                    LimelightHelpers.getBotPose2d_wpiRed(this.name) :
                    LimelightHelpers.getBotPose2d_wpiBlue(this.name);
            double latency = LimelightHelpers.getLatency_Capture(this.name) +
                    LimelightHelpers.getLatency_Pipeline(this.name);
            return new VisionTypes.PoseInfo(pose, latency);
        } else {
            DataLogManager.log("[Limelight] No alliance information available from DriverStation");
            throw new RuntimeException("No alliance information available");
        }
    }

    public void setCameraMode(VisionTypes.CameraMode cameraMode) {
        Consumer<String> selector = switch (cameraMode) {
            case ImageProcessing -> LimelightHelpers::setCameraMode_Processor;
            case DriverCamera -> LimelightHelpers::setCameraMode_Driver;
        };
        selector.accept(this.name);
    }


    public void setLEDMode(VisionTypes.LEDMode ledMode) {
        Consumer<String> selector = switch (ledMode) {
            case CurrentPipeline -> LimelightHelpers::setLEDMode_PipelineControl;
            case ForceOff -> LimelightHelpers::setLEDMode_ForceOff;
            case ForceBlink -> LimelightHelpers::setLEDMode_ForceBlink;
            case ForceOn -> LimelightHelpers::setLEDMode_ForceOn;
        };
        selector.accept(this.name);
    }
}
