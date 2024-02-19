package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

public class FeederHelper implements Sendable {
    private boolean enabled = false;
    private final String name;
    private final TalonFX motor;
    private final DigitalInput noteStopSensor;

    private final DutyCycleOut openLoopOut = new DutyCycleOut(0.0);
    private double openLoopSetpoint = -0.50;

    private Timer shooterTimer;

    public FeederHelper(String parent, String name, TalonFX motor, DigitalInput noteStopSensor) {
        this.name = name;
        this.motor = motor;
        this.noteStopSensor = noteStopSensor;

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.withOpenLoopRamps(
                new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(0.0))
                .withCurrentLimits(
                        new CurrentLimitsConfigs().withSupplyCurrentLimit(40)
                                .withSupplyCurrentLimitEnable(true));
        this.motor.getConfigurator().apply(config);

        shooterTimer = new Timer();
        shooterTimer.start();
        shooterTimer.reset();
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
        double out = 0;
        if(shooterTimer.get() < .125) {
            out = -1.0;
        }
        else {
            out = 0.0;
        }
        motor.setControl(openLoopOut.withOutput(out));
    }

    public void shoot() {
        shooterTimer.reset();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty(name + "/Enabled", this::isEnabled, this::setEnabled);
        builder.addDoubleProperty(name + "/Open Loop Setpoint", this::getOpenLoopSetpoint, this::setOpenLoopSetpoint);
    }
}