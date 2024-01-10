package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems;

public class ScorePositionDetector {
    private TargetInfo currentTarget;

    public ScorePositionDetector() {
        SmartDashboard.setDefaultNumber("ScorePositionDetector/xThreshold", 0.5);
        SmartDashboard.setDefaultNumber("ScorePositionDetector/dThreshold", 0.5);
    }

    public void setCurrentTarget(TargetInfo targetInfo) {
        this.currentTarget = targetInfo;
        SmartDashboard.putString("ScorePositionDetector/CurrentTarget", this.currentTarget.name());
    }

    public boolean inRequestedScoringPosition() {
        if (currentTarget == null) {
            return false;
        }

        double xThreshold = SmartDashboard.getNumber("ScorePositionDetector/xThreshold", 0.5);
        double dThreshold = SmartDashboard.getNumber("ScorePositionDetector/dThreshold", 0.5);

        double targetDistance = currentTarget.distance;
        var visionInfo = Subsystems.visionSubsystem.getVisionInfo();

        var currentX = Math.abs(visionInfo.xOffset);
        var currentD = visionInfo.distanceToTarget;

        if ((currentX < xThreshold) && (Math.abs(currentD - targetDistance) < dThreshold)) {
            return true;
        }
        return false;
    }
}
