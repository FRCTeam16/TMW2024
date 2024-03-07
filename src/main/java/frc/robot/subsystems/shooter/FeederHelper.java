package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class FeederHelper implements Sendable {
    private boolean enabled = true;
    private final String name;
    private final TalonFX motor;
    private final DigitalInput noteStopSensor;
    private final DigitalInput slowSpeedSensor;
    private boolean slowFeedMode = false;

    private final VoltageOut openLoopOut = new VoltageOut(0.0);
    private double openLoopSetpoint = 0.0;
    private double shootingSpeed = -6;

    private Timer shooterTimer;
    private double feedShooterSpeed = -1.5; // WARNING: Must change Intake feed
    private double slowFeedShooterSpeed = -1.5;

    boolean shooting = false;   // whether we are shooting


    public FeederHelper(String parent, String name, TalonFX motor, DigitalInput noteStopSensor, DigitalInput slowSpeedSensor) {
        this.name = name;
        this.motor = motor;
        this.noteStopSensor = noteStopSensor;
        this.slowSpeedSensor = slowSpeedSensor;

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
                out = -12;
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
        // drop out of shooting mode
        shooting = false;

        // while we have not tripped out sensor
        if (!isNoteDetected()) {
            this.enabled = true;
            if (!isSlowSensorDetected()) {
                this.openLoopSetpoint = slowFeedMode ? this.slowFeedShooterSpeed : this.feedShooterSpeed;  
            } else {
                // Switch to slow speed mode when we see the note
                slowFeedMode = true;
                this.openLoopSetpoint = this.slowFeedShooterSpeed;
            }
        } else {
            // Turn off feed when we detect the note
            this.enabled = false;
            this.slowFeedMode = false;
        }
    }

    public boolean isNoteDetected() {
        return !noteStopSensor.get();
    }

    public boolean isSlowSensorDetected() {
        return !slowSpeedSensor.get();
    }

    public double getSlowFeedShooterSpeed() {
        return slowFeedShooterSpeed;
    }

    public boolean isSlowFeedMode() {
        return slowFeedMode;
    }

    public void setSlowFeedMode(boolean slowFeedMode) {
        this.slowFeedMode = slowFeedMode;
    }

    public void setSlowFeedShooterSpeed(double slowFeedShooterSpeed) {
        this.slowFeedShooterSpeed = slowFeedShooterSpeed;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty(name + "/Enabled", this::isEnabled, this::setEnabled);
        builder.addBooleanProperty(name + "/Note Detected", this::isNoteDetected, null);
        builder.addDoubleProperty(name + "/Open Loop Setpoint", this::getOpenLoopSetpoint, this::setOpenLoopSetpoint);
        builder.addBooleanProperty(name + "/Slow Sensor", this::isSlowSensorDetected, null);
        if (Constants.Dashboard.ConfigurationMode || Constants.Dashboard.ShooterConfigMode) {
            builder.addDoubleProperty(name + "/Feed Note Speed", this::getFeedShooterSpeed, this::setFeedShooterSpeed);
            builder.addDoubleProperty(name + "/Shoot Speed", this::getShootingSpeed, this::setShootingSpeed);
            builder.addDoubleProperty(name + "/Slow Feed Speed", this::getSlowFeedShooterSpeed, this::setSlowFeedShooterSpeed);
        }
    }
}