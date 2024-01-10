package frc.robot.subsystems.DMS;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.function.DoubleToIntFunction;

public class DMSStats {
    private static final double VEL_THRESHOLD = 0.85;
    private static final double AMP_THRESHOLD = 1.4;

    public DriveInfo<Double> current = new DriveInfo<>(0.0);
    public DriveInfo<Double> velocity = new DriveInfo<>(0.0);
    private int currentLoops = 0;
    private int veloLoops = 0;


    public void addDriveCurrent(DriveInfo<Double> driveOutputCurrent) {
        currentLoops++;
        current.FL = current.FL + Math.abs(driveOutputCurrent.FL) / currentLoops;
        current.FR = current.FR + Math.abs(driveOutputCurrent.FR) / currentLoops;
        current.RL = current.RL + Math.abs(driveOutputCurrent.RL) / currentLoops;
        current.RR = current.RR + Math.abs(driveOutputCurrent.RR) / currentLoops;
    }

    public void addDriveVelocity(DriveInfo<Double> driveVelocity) {
        veloLoops++;
        velocity.FL = velocity.FL + Math.abs(driveVelocity.FL) / veloLoops;
        velocity.FR = velocity.FR + Math.abs(driveVelocity.FR) / veloLoops;
        velocity.RL = velocity.RL + Math.abs(driveVelocity.RL) / veloLoops;
        velocity.RR = velocity.RR + Math.abs(driveVelocity.RR) / veloLoops;
    }

    public static void print(String label, DriveInfo<?> info) {
        System.out.println(label + 
            " FR: " + info.FR +
            " FL: " + info.FL + 
            " RL: " + info.RL + 
            " RR: " + info.RR);
    }

    public static double average(DriveInfo<Double> info) {
        return (info.FL + info.FR + info.RL + info.RR) / 4.0;
    }

    public DriveInfo<Integer> calculateStatus() {
        DriveInfo<Integer> status = new DriveInfo<>(0);

        double velAvg = DMSStats.average(velocity);
        double ampAvg = DMSStats.average(current);
        System.out.println("Vel Avg: " + velAvg + " | Amp Avg: " + ampAvg);
        status.FL = calc(velocity.FL, velAvg, current.FL, ampAvg);
        status.FR = calc(velocity.FR, velAvg, current.FR, ampAvg);
        status.RL = calc(velocity.RL, velAvg, current.RL, ampAvg);
        status.RR = calc(velocity.RR, velAvg, current.RR, ampAvg);
        return status;
    }

    private int calc(double vel, double velAvg, double amp, double ampAvg) {
        if (amp == 0) {
            return 4;
        }
        if (vel == 0) {
            return 5;
        }
        boolean velOutside = (vel / velAvg) < VEL_THRESHOLD;
        boolean ampOutside = (amp / ampAvg) > AMP_THRESHOLD;

        if (!velOutside && !ampOutside) {
            return 1; // good
        } else if (!velOutside && ampOutside) {
            return 2; // amp outside
        } else if (velOutside && !ampOutside) {
            return 3;
        } else {
            return 4;   // both out of range
        }
    }

    /**
     * https://www.itl.nist.gov/div898/handbook/eda/section3/eda35h.htm
     * @return
     */
    public DriveInfo<Integer> calculateStatusMAD() {
        DriveInfo<Integer> status = new DriveInfo<>(0);

        double[] modifiedZCurrent = calculateModifiedZ(List.of(current.FL, current.FR, current.RL, current.RR));
        double[] modifiedZVelocity = calculateModifiedZ(List.of(velocity.FL, velocity.FR, velocity.RL, velocity.RR));

        double MAD_THRESHOLD = 3.5; // recommended as outlier
        DoubleToIntFunction dif = (c -> Math.abs(c) < MAD_THRESHOLD ? 1 : 2);
        int[] currentStatus = Arrays.stream(modifiedZCurrent).mapToInt(dif).toArray();
        int[] velocityStatus = Arrays.stream(modifiedZVelocity).mapToInt(dif).toArray();


        status.FL = calculateStatus(currentStatus[0], velocityStatus[0]);
        status.FR = calculateStatus(currentStatus[1], velocityStatus[1]);
        status.RL = calculateStatus(currentStatus[2], velocityStatus[2]);
        status.RR = calculateStatus(currentStatus[3], velocityStatus[3]);

        return status;
    }

    private double[] calculateModifiedZ(List<Double> data) {
        Collections.sort(data);
        // assume even number of observations-
        double median = (data.get(1) + data.get(2)) / 2;
        double[] deviations = data.stream().mapToDouble(c -> c - median).toArray();
        double medianDeviation = (deviations[1] + deviations[2]) / 2;

        double[] modifiedZ = data.stream().mapToDouble(c -> (0.6745 * (c - median) ) / medianDeviation).toArray();
        return modifiedZ;
    }

    private int calculateStatus(int currentStatus, int velocityStatus) {
        if (currentStatus == 0 && velocityStatus == 0) {
            return 0;
        } else if (currentStatus == 0 && velocityStatus == 1) {
            return 1;
        } else if (currentStatus == 1 && velocityStatus == 0) {
            return 2;
        } else if (currentStatus == 1 && velocityStatus == 1) {
            return 3;
        } else {
            return 4;
        }

    }
}