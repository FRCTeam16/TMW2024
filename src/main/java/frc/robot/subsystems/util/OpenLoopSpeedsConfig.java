package frc.robot.subsystems.util;

public class OpenLoopSpeedsConfig {
    private double upSpeed = .5;
    private double downSpeed = -.5;


    public OpenLoopSpeedsConfig(double upSpeed, double downSpeed) {
        this.upSpeed = upSpeed;
        this.downSpeed = downSpeed;
    }

    public double getUpSpeed() {
        return upSpeed;
    }

    public void setUpSpeed(double upSpeed) {
        this.upSpeed = upSpeed;
    }

    public double getDownSpeed() {
        return downSpeed;
    }

    public void setDownSpeed(double downSpeed) {
        this.downSpeed = downSpeed;
    }
}
