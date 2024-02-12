package frc.robot.subsystems.util;


public class DoubleChanger {

    private boolean changed = false;
    private double epsilon = 0.0001; // you can set it to any small value you'd like

    /**
     * Updates the value with the new value if the difference between them exceeds a certain threshold.
     *
     * @param oldValue the old value
     * @param newValue the new value
     * @return the updated value
     */
    public double change(double oldValue, double newValue) {
        if (Math.abs(oldValue - newValue) > epsilon) { // use epsilon in the comparison
            changed = true;
            return newValue;
        } else {
            return oldValue;
        }
    }

    /**
     * Checks if any of the values checked have been changed.
     *
     * @return true if a value has been changed, false otherwise
     */
    public boolean isChanged() {
        return changed;
    }

    public void setEpsilon(double epsilon) {
        this.epsilon = epsilon;
    }

    public double getEpsilon() {
        return epsilon;
    }

    public void reset() {
        this.changed = false;
    }

}