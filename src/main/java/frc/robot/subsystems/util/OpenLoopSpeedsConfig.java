package frc.robot.subsystems.util;

/**
 * Represents the open-loop speeds for intake pivot movement.
 */
public class OpenLoopSpeedsConfig {
    private double upSpeed = .5;
    private double downSpeed = -.5;

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
