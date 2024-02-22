package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.util.PIDHelper;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.VisionTypes;

import java.util.Optional;

public class VisionAimManager {
    private final Limelight limelight;

    private Counter missingTargetCounter = new Counter().withThreshold(50);
    private VisionAimResult lastTarget = null;


    public VisionAimManager(Limelight limelight) {
        this.limelight = limelight;
    }


    public Optional<VisionAimResult> calculate() {
        VisionTypes.TargetInfo targetInfo = limelight.getTargetInfo();
        if (targetInfo.hasTarget()) {
            // TODO: Add less than 75" shoot profile
            final double distance = targetInfo.calculateDistance();
            final double angle = 78.3 - (0.711 * distance) + (0.0022 * distance * distance);
            final double bottomSpeed = (0.241 * distance) + 7.71;
            final double topSpeed = (0.554 * distance) - 12.2;

            VisionAimResult value = new VisionAimResult(targetInfo, new ShootingProfile(angle, topSpeed, bottomSpeed));
            SmartDashboard.putNumber("VisionAimManager/Distance", value.targetInfo.calculateDistance());
            SmartDashboard.putNumber("VisionAimManager/Xoffset", targetInfo.xOffset());
            SmartDashboard.putNumber("VisionAimManager/bottomSpeed", bottomSpeed);
            SmartDashboard.putNumber("VisionAimManager/topSpeed", topSpeed);
            SmartDashboard.putNumber("VisionAimManager/angle", angle);
            return Optional.of(value);
        } else {
            if (missingTargetCounter.increment()) {
                lastTarget = null;
            }
            return Optional.ofNullable(lastTarget);
        }
    }

    public record VisionAimResult(VisionTypes.TargetInfo targetInfo, ShootingProfile shootingProfile) {
    }

    public record ShootingProfile(double pivotAngle, double upperSpeed, double lowerSpeed) {}

    static class Counter {
        private int count = 0;
        private int threshold = 5;

        Counter withThreshold(int threshold) {
            this.threshold = threshold;
            return this;
        }

        void reset() {
            count = 0;
        }

        boolean increment() {
           return (count++) >= threshold;
        }
    }
}
