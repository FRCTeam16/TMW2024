package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class FeederHelper implements Sendable {
    private boolean enabled = false;
    private final String name;
    private final TalonFX motor;

    private final DutyCycleOut openLoopOut = new DutyCycleOut(0.0);
    private double openLoopSetpoint = 0.0;

    public FeederHelper(String parent, String name, TalonFX motor) {
        this.name = name;
        this.motor = motor;
    }

    public void setOpenLoopSetpoint(double openLoopSetpoint) {
        this.openLoopSetpoint = openLoopSetpoint;
    }

    public double getOpenLoopSetpoint() {
        return openLoopSetpoint;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void periodic() {
        double out = enabled ? openLoopSetpoint : 0.0;
        motor.setControl(openLoopOut.withOutput(out));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty(name + "/Enabled", this::isEnabled, this::setEnabled);
        builder.addDoubleProperty(name + "/Open Loop Setpoint", this::getOpenLoopSetpoint, this::setOpenLoopSetpoint);
    }
}