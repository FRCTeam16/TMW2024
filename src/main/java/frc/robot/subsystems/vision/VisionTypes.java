package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

public class VisionTypes {
    /**
     * Camera Modes
     */
    public enum CameraMode {
        ImageProcessing(0),
        DriverCamera(1);

        private final int mode;

        CameraMode(int mode) {
            this.mode = mode;
        }
    }

    /**
     * LED modes
     */
    public enum LEDMode {
        CurrentPipeline(0),
        ForceOff(1),
        ForceBlink(2),
        ForceOn(3);

        private final int mode;

        LEDMode(int mode) {
            this.mode = mode;
        }
    }

    public record TargetInfo(boolean hasTarget, double xOffset, double yOffset, double latency) {
        public double calculateDistance(double heightToCamera, double heightToTarget, double cameraAngle) {
            if (this.hasTarget) {
                double goalRadians = Units.degreesToRadians(cameraAngle + this.yOffset);
                return (heightToTarget - heightToCamera) / Math.tan(goalRadians);
            } else {
                return -1;
            }
        }
    }

    public record PoseInfo(Pose2d pose, double latency) {
    }

    public record LimelightInfo(String name, double heightToCamera, double cameraAngle) {}
}
