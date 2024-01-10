package frc.robot.subsystems.util;

public class BSMath {

    /**
     * Maps a value from one linear system to another
     * NewValue = (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin
     */
    public static double map(double oldValue, double oldMin, double oldMax, double newMin, double newMax ) {
        return  (((oldValue - oldMin) * (newMax - newMin)) / (oldMax - oldMin)) + newMin;
    }
}
