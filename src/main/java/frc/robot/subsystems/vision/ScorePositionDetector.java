package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ScorePositionDetector {
    private double targetDistance;

    public ScorePositionDetector() {
        SmartDashboard.setDefaultNumber("ScorePositionDetector/xThreshold", 0.5);
        SmartDashboard.setDefaultNumber("ScorePositionDetector/dThreshold", 0.5);
    }

    public void setCurrentTarget(double targetDistance) {
        this.targetDistance = targetDistance;
        SmartDashboard.putNumber("ScorePositionDetector/CurrentTarget", this.targetDistance);
    }

    public boolean inRequestedScoringPosition(VisionTypes.TargetInfo targetInfo, VisionTypes.LimelightInfo limelightInfo) {
        double xThreshold = SmartDashboard.getNumber("ScorePositionDetector/xThreshold", 0.5);
        double dThreshold = SmartDashboard.getNumber("ScorePositionDetector/dThreshold", 0.5);


        var currentX = Math.abs(targetInfo.xOffset());
        var currentD = targetInfo.calculateDistance();

        if ((currentX < xThreshold) && (Math.abs(currentD - targetDistance) < dThreshold)) {
            return true;
        }
        return false;
    }
}
