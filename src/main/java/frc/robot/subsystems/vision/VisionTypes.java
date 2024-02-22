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

    public record CameraDistanceValues(double heightToCamera, double heightToTarget, double cameraAngle) {}

    public record TargetInfo(boolean hasTarget, double xOffset, double yOffset, double latency, VisionTypes.CameraDistanceValues cameraDistanceValues) {

        public double calculateDistance() {
            return calculateDistance(this.cameraDistanceValues.heightToTarget);
        }

        public double calculateDistance(double heightToTarget) {
            if (this.hasTarget) {
                double goalRadians = Units.degreesToRadians(this.cameraDistanceValues.cameraAngle + this.yOffset);
                return (heightToTarget - this.cameraDistanceValues.heightToCamera) / Math.tan(goalRadians);
            } else {
                return -1;
            }
        }
    }

    public record PoseInfo(Pose2d pose, double latency) {
    }

    public record LimelightInfo(String name, double heightToCamera, double cameraAngle) {}
}
