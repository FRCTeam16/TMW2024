package frc.robot.subsystems.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.VisionTypes;

public class VisionAlignmentHelper {
    private static final double DEFAULT_KP = 0.02;
    private PIDController pid = new PIDController(DEFAULT_KP, 0, 0);
    private PIDHelper pidHelper = new PIDHelper("VisionAlign");
    private double tolerance = 1.0;
    private double maxSpeed = 0.3;

    public VisionAlignmentHelper(Limelight limelight) {
        pidHelper.initialize(DEFAULT_KP, 0, 0, 0, 0, 0);
        this.pid.setTolerance(tolerance);
        this.pid.setIntegratorRange(-1.0, 1.0);
    }
    public VisionAlignmentHelper() {
        this(null);
    }

    public VisionAlignmentHelper overrideMaxSpeed(double speed) {
        this.maxSpeed = speed;
        return this;
    }

    public VisionAlignmentHelper withP(double pValue) {
        this.pidHelper.overrideP(pValue);
        return this;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public double calculate(VisionTypes.TargetInfo targetInfo) {
        pidHelper.updateValuesFromDashboard();
        pidHelper.updatePIDController(this.pid);
        double output = 0.0;
        if (targetInfo.hasTarget()) {
            output = this.pid.calculate(targetInfo.xOffset(), 0);
        }
        double clampVal = maxSpeed;
        double clampedValue = MathUtil.clamp(output, -clampVal, clampVal);
        return clampedValue;
    }

    public boolean inPosition() {
        return this.pid.atSetpoint();
    }
}
