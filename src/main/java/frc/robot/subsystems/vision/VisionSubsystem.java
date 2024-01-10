package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.vision.Limelight.CameraMode;
import frc.robot.subsystems.vision.Limelight.LEDMode;
import frc.robot.subsystems.vision.Limelight.SceneInfo;


/**
 * Vision subsystem.  
 */
public class VisionSubsystem extends SubsystemBase implements Lifecycle {
  private final Limelight limelight;
  private VisionInfo visionInfo = new VisionInfo();
  private ScorePositionDetector scorePosotionDetector = new ScorePositionDetector();

  private final double CAMERA_HEIGHT_IN;
  private final double TARGET_HEIGHT_IN;
  private final double CAMERA_ANGLE_DEGREES;


  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    limelight = new Limelight();
    double defaultCameraHeight = 24.5;
    double defaultTargetHeight = TargetInfo.LowPole.height;
    double defaultCameraAngle = -1.0;
    
    SmartDashboard.setDefaultNumber("Vision/CameraHeightInches", defaultCameraHeight);
    SmartDashboard.setDefaultNumber("Vision/TargetHeightInches", defaultTargetHeight);
    SmartDashboard.setDefaultNumber("Vision/CameraAngleDegrees", defaultCameraAngle);
    CAMERA_HEIGHT_IN = SmartDashboard.getNumber("Vision/CameraHeightInches", defaultCameraHeight);
    TARGET_HEIGHT_IN = SmartDashboard.getNumber("Vision/TargetHeightInches", defaultTargetHeight);
    CAMERA_ANGLE_DEGREES = SmartDashboard.getNumber("Vision/CameraAngleDegrees", defaultCameraAngle);
  }

  public Limelight getLimelight() {
    return limelight;
  }

  public VisionInfo getVisionInfo() {
    return visionInfo;
  }

  public void enable() {
    limelight.setCameraMode(CameraMode.ImageProcessing);
    limelight.setLEDMode(LEDMode.CurrentPipeline);
  }

  public void disable() {
    limelight.setLEDMode(LEDMode.ForceOff);
  }

  @Override
  public void periodic() {
    var sceneInfo = limelight.getScene();
    var aprilInfo = limelight.getAprilInfo();
    
    // Update out information
    visionInfo = new VisionInfo();
    visionInfo.hasTarget = sceneInfo.hasTarget;
    visionInfo.xOffset = sceneInfo.xOffset;
    visionInfo.yOffset = sceneInfo.yOffset;
    visionInfo.targetArea = sceneInfo.targetArea;

    visionInfo.targetId = aprilInfo.targetId;
    
    double distance_inches = -1.0;
    if (visionInfo.hasTarget) {
      double targetHeight = TargetInfo.LowPole.height;
      Pipeline currentPipeline = Pipeline.findPipeline(limelight.getCurrentPipelin());
      if (Pipeline.April == currentPipeline) {
        targetHeight = TargetInfo.AprilTag.height;
      }
      SmartDashboard.putNumber("Vision/TargetHeightInches", targetHeight);

      double cameraAngle = SmartDashboard.getNumber("Vision/CameraAngleDegrees", CAMERA_ANGLE_DEGREES);   // CAMERA_ANGLE_DEGREES
      distance_inches = CalculateDistance(CAMERA_HEIGHT_IN, targetHeight, cameraAngle, visionInfo.yOffset);
    }
    visionInfo.distanceToTarget = distance_inches;
    
    SmartDashboard.putNumber("Vision/DistToTargetInches", visionInfo.distanceToTarget);
  }

  /**
   * Calculates the distance to the currently observed target, or -1 if no target
   * 
   * @param heightToCamera
   * @param heightToTarget
   * @return  
   */
  public double CalculateDistance(double heightToCamera, double heightToTarget, double cameraAngle, double yOffset) {
    if (this.visionInfo.hasTarget) {
      double goalRadians = Units.degreesToRadians(cameraAngle + yOffset);
      return (heightToTarget - heightToCamera) / Math.tan(goalRadians);
    } else {
      return -1;
    }
}

  /**
   * Vision Information
   */
  public static class VisionInfo extends SceneInfo {
    public double distanceToTarget = -1;
  }

  public void selectPipelineSync(Pipeline pipeline) {
    limelight.setCurrentPipeline(pipeline.pipelineNumber);
  }

  public Command selectPipeline(Pipeline pipeline) {
    return Commands.runOnce(() -> limelight.setCurrentPipeline(pipeline.pipelineNumber))
      .ignoringDisable(true);
  }

  public ScorePositionDetector getScorePositionDetector() {
    return this.scorePosotionDetector;
  }
}
