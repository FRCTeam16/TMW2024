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
    private boolean enabled = true;
    private final String name;
    private final TalonFX motor;
    private final DigitalInput noteStopSensor;

    private final DutyCycleOut openLoopOut = new DutyCycleOut(0.0);
    private double openLoopSetpoint = 0.0;
    private double shootingSpeed = -0.50;

    private Timer shooterTimer;
    private double feedShooterSpeed = -0.1; // WARNING: Must change Intake feed

    boolean shooting = false;   // whether we are shooting


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
        if (shooting) {
            if( shooterTimer.get() < .125) {
                out = -1.0;
            } else {
                openLoopSetpoint = 0.0;
                shooting = false;
            }
        }
        else if (enabled) {
            out = openLoopSetpoint;
        }
        motor.setControl(openLoopOut.withOutput(out));
    }

    public void shoot() {
        shooting = true;
        openLoopSetpoint = shootingSpeed;
        shooterTimer.reset();
    }

    public double getFeedShooterSpeed() {
        return this.feedShooterSpeed;
    }

    public void setFeedShooterSpeed(double feedShooterSpeed) {
        this.feedShooterSpeed = feedShooterSpeed;
    }

    public double getShootingSpeed() {
        return shootingSpeed;
    }

    public void setShootingSpeed(double shootingSpeed) {
        this.shootingSpeed = shootingSpeed;
    }

    public void receiveFromIntake() {
        shooting = false;
        if(!isNoteDetected()) {
            this.openLoopSetpoint = this.feedShooterSpeed;
            this.enabled = true;
        }
        else {
            this.enabled = false;
        }
    }

    public boolean isNoteDetected() {
        return !noteStopSensor.get();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty(name + "/Enabled", this::isEnabled, this::setEnabled);
        builder.addDoubleProperty(name + "/Open Loop Setpoint", this::getOpenLoopSetpoint, this::setOpenLoopSetpoint);
        builder.addDoubleProperty(name + "/Feed Note Speed", this::getFeedShooterSpeed, this::setFeedShooterSpeed);
        builder.addDoubleProperty(name + "/Shoot Speed", this::getShootingSpeed, this::setShootingSpeed);
        builder.addBooleanProperty(name + "/Note Detected", this::isNoteDetected, null);
    }
}