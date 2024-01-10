package frc.robot.subsystems.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems;
import frc.robot.subsystems.vision.VisionSubsystem.VisionInfo;

// FIXME: Needs tsting for 2024
public class VisionAlignmentHelper {
    private static final double DEFAULT_KP = 0.04;
    private PIDController pid = new PIDController(DEFAULT_KP, 0, 0);
    private PIDHelper pidHelper = new PIDHelper("VisionAlign");
    private double tolerance = 1.0;
    private double maxSpeed = 0.3;


    public VisionAlignmentHelper() {
        pidHelper.initialize(DEFAULT_KP, 0, 0, 0, 0, 0);
        this.pid.setTolerance(tolerance);
        this.pid.setIntegratorRange(-1.0, 1.0);
        SmartDashboard.setDefaultNumber("VisionAlign/maxSpeed", maxSpeed);
    }

    public VisionAlignmentHelper overrideMaxSpeed(double speed) {
        this.maxSpeed = speed;
        return this;
    }

    public VisionAlignmentHelper withP(double pValue) {
        System.out.println("visionhelper p override = " + pValue);
        this.pidHelper.overrideP(pValue);
        return this;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public double calculate() {
        pidHelper.updateValuesFromDashboard();
        pidHelper.updatePIDController(this.pid);
        VisionInfo visionInfo = Subsystems.visionSubsystem.getVisionInfo();
        double output = 0.0;
        if (visionInfo.hasTarget) {
            output = this.pid.calculate(visionInfo.xOffset, 0);
        }
        // double clampVal = SmartDashboard.getNumber("VisionAlign/maxSpeed", maxSpeed);
        double clampVal = maxSpeed;        
        double clampedValue = MathUtil.clamp(output, -clampVal, clampVal);
        return clampedValue;
    }

    public boolean inPosition() { 
        return this.pid.atSetpoint();
    }
}
