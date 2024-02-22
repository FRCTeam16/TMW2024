package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.util.PIDHelper;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.VisionTypes;

import java.util.Optional;

public class VisionAimManager {
    private final Limelight limelight;
    private final PIDController pid;
    private final PIDHelper pidHelper = new PIDHelper("ShooterSubsystem/VisionAimManager/PID");

    private Counter missingTargetCounter = new Counter().withThreshold(50);
    private VisionAimResult lastTarget = null;


    public VisionAimManager(Limelight limelight) {
        this.limelight = limelight;
        this.pidHelper.initialize(0, 0, 0, 0, 0, 0);
        this.pid = new PIDController(0, 0, 0);
        this.pid.setSetpoint(0);

        pidHelper.updatePIDController(pid);
    }

    public VisionAimManager withTolerance(double degrees) {
        this.pid.setTolerance(degrees);
        return this;
    }

    public Optional<VisionAimResult> calculate() {
        VisionTypes.TargetInfo targetInfo = limelight.getTargetInfo();
        if (targetInfo.hasTarget()) {
            double output = pid.calculate(targetInfo.yOffset());
            VisionAimResult value = new VisionAimResult(targetInfo, output);
            SmartDashboard.putNumber("VisionAimManager/Distance", value.targetInfo.calculateDistance());
            return Optional.of(value);
        } else {
            if (missingTargetCounter.increment()) {
                lastTarget = null;
            }
            return Optional.ofNullable(lastTarget);
        }
    }

    public record VisionAimResult(VisionTypes.TargetInfo targetInfo, double output) {
    }

    public record ShootingProfile(double pivotAngle, double upperSpeed, double lowerSpeed) {}

    static class Counter {
        private int count = 0;
        private int threshold = 5;

        Counter withThreshold(int threshold) {
            this.threshold = threshold;
            return this;
        }

        void reset() {
            count = 0;
        }

        boolean increment() {
           return (count++) >= threshold;
        }
    }
}
