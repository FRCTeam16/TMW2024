package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.util.Counter;
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


    public ShootingProfile getShortShot() {
        return new ShootingProfile(55, 25, 25);
    }

    /**
     * Calculate the vision aim result using the limelight and computed shooting profiles
     * @return
     */
    public Optional<VisionAimResult> calculate() {
        VisionTypes.TargetInfo targetInfo = limelight.getTargetInfo();
        if (targetInfo.hasTarget()) {
            // TODO: Add less than 75" shoot profile
            final double distance = targetInfo.calculateDistance();
            final double shooterSpeed = (0.2 * distance) + 24.9;
            final double angle = 61.1 - (0.409 * distance) + (0.000897 * distance * distance);

            VisionAimResult value = new VisionAimResult(targetInfo, new ShootingProfile(angle, shooterSpeed, shooterSpeed));
            SmartDashboard.putNumber("VisionAimManager/Distance", value.targetInfo.calculateDistance());
            SmartDashboard.putNumber("VisionAimManager/Xoffset", targetInfo.xOffset());
            SmartDashboard.putNumber("VisionAimManager/bottomSpeed", shooterSpeed);
            SmartDashboard.putNumber("VisionAimManager/topSpeed", shooterSpeed);
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

}
