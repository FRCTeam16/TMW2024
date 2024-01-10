package frc.robot.subsystems.vision;

public enum TargetInfo {
    HighPole(43.25, 52),
    LowPole(23.5, 36.5),
    AprilTag(18.25, 24);

    public final double height;
    public final double distance;

    private TargetInfo(double height, double distance) { 
        this.height = height; 
        this.distance = distance;
    }
}
