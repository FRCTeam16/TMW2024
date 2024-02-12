package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.util.PIDHelper;

public class ShooterHelper implements Sendable {
    private final String parentName;
    boolean updateConfig = true;    // whether to update the config every cycle
    private final PIDHelper pid;
    private final String name;
    private final TalonFX motor;
    private final TalonFXConfiguration config;

    private boolean enabled = false;
    private boolean openLoop = true;
    private double openLoopSetpoint = 0.0;
    private double velocitySetpoint = 0.0;

    private boolean invert = false;

    private DutyCycleOut openLoopOut = new DutyCycleOut(0);
    private VelocityVoltage velocityOut = new VelocityVoltage(0).withSlot(0);

    // 6000 RPM, 12V battery
    // https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/VelocityClosedLoop/src/main/java/frc/robot/Robot.java
    // https://pro.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/basic-pid-control.html#velocity-control
    private static final double kraken_kv = 1 / ((6000 / 12.0) / 60.0);
    private static final double kraken_kA = 0.01; // 1 rps acceleration requires 0.01 V
    private static final double kraken_kS = 0.00;   // how many volts to overcome static friction


    public ShooterHelper(String parentName, String name, TalonFX motor) {
        this.parentName = parentName;
        this.name = name;
        this.motor = motor;

        this.pid = new PIDHelper(parentName + "/" + name);
        this.pid.initialize(0.04,0,0,0,kraken_kv,kraken_kA);

        config = new TalonFXConfiguration();
        config.Slot0.withKS(kraken_kS);   // overcome static friction

        pid.updateConfiguration(config.Slot0);

        setMotorPID();
    }

    /**
     * Sets the PID values for the motor based on the config passed in
     */
    private void setMotorPID() {
        StatusCode code = this.motor.getConfigurator().apply(config);
        if (!code.isOK()) {
            DataLogManager.log("ShooterHelper(" + this.name + ") problem: " + code.getDescription());
        }
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void setOpenLoop(boolean openLoop) {
        this.openLoop = openLoop;
    }

    public boolean isOpenLoop() {
        return openLoop;
    }

    public void setInvert(boolean invert) {
        this.invert = invert;
    }

    public boolean isInvert() {
        return invert;
    }

    public double getOpenLoopSetpoint() {
        return openLoopSetpoint;
    }

    public void setOpenLoopSetpoint(double openLoopSetpoint) {
        this.openLoopSetpoint = openLoopSetpoint;
    }

    public double getVelocitySetpoint() {
        return velocitySetpoint;
    }

    public void setVelocitySetpoint(double velocitySetpoint) {
        this.velocitySetpoint = velocitySetpoint;
    }

    private int direction() {
        return (invert) ? -1 : 1;
    }

    void periodic() {
        if (enabled) {
            if (openLoop) {
                this.motor.setControl(openLoopOut.withOutput(direction() * openLoopSetpoint));
            } else {
                if (pid.updateValuesFromDashboard()) {
                    // TODO test this doesn't interfere pid.updateConfiguration(config);
                    // this.setMotorPID();
                }
                this.motor.setControl(velocityOut.withVelocity(direction() * velocitySetpoint).withFeedForward(pid.kF).withEnableFOC(true));
            }
        }
        else {
            this.motor.setControl(openLoopOut.withOutput(0));
        }

        SmartDashboard.putNumber(parentName + "/" + name + "/ActualVelocity", this.motor.getVelocity().getValue());
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty(this.name + "/Enabled", this::isEnabled, this::setEnabled);
        builder.addBooleanProperty(this.name + "/OpenLoop", this::isOpenLoop, this::setOpenLoop);
        builder.addDoubleProperty(this.name + "/Open Loop Setpoint", this::getOpenLoopSetpoint, this::setOpenLoopSetpoint);
        builder.addDoubleProperty(this.name + "/Velocity Setpoint", this::getVelocitySetpoint, this::setVelocitySetpoint);
    }
}
