package frc.robot.subsystems.util;

public class Counter {
    private int count = 0;
    private int threshold = 5;

    public Counter withThreshold(int threshold) {
        this.threshold = threshold;
        return this;
    }

    public boolean isThresholdMet() {
        return (count >= threshold);
    }

    public void reset() {
        count = 0;
    }

    public boolean increment() {
        return (count++) >= threshold;
    }
}
