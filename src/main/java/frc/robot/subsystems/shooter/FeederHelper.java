package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class FeederHelper implements Sendable {
    private final String name;
    private final TalonFX motor;
    private final DigitalInput noteStopSensor;
    private final DigitalInput slowSpeedSensor;
    private final VoltageOut openLoopOut = new VoltageOut(0.0);
    boolean shooting = false;   // whether we are shooting
    boolean velocityMode = false;
    private boolean enabled = true;
    private boolean slowFeedMode = false;
    private double openLoopSetpoint = 0.0;
    private double shootingSpeed = -6;
    private final Timer shooterTimer;
    private double feedShooterSpeed = -1.5; // WARNING: Must change Intake feed
    private double slowFeedShooterSpeed = -1.5;
    private VelocityVoltage velocityVoltage = new VelocityVoltage(0);
    private VelocityControl velocityControl = new VelocityControl();


    public FeederHelper(String parent, String name, TalonFX motor, DigitalInput noteStopSensor, DigitalInput slowSpeedSensor) {
        this.name = name;
        this.motor = motor;
        this.noteStopSensor = noteStopSensor;
        this.slowSpeedSensor = slowSpeedSensor;

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.withSlot0(
                new Slot0Configs()
                        .withKP(0.8)
                        .withKV(0.15)
        ).withOpenLoopRamps(
                new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(0.0)
        ).withCurrentLimits(
                new CurrentLimitsConfigs().withSupplyCurrentLimit(40)
                        .withSupplyCurrentLimitEnable(true));
        this.motor.getConfigurator().apply(config);

        shooterTimer = new Timer();
        shooterTimer.start();
        shooterTimer.reset();
    }

    public double getOpenLoopSetpoint() {
        return openLoopSetpoint;
    }

    public void setOpenLoopSetpoint(double openLoopSetpoint) {
        this.openLoopSetpoint = openLoopSetpoint;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public void enableBreakMode() {
        this.motor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void enableCoastMode() {
        this.motor.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setVelocityMode(boolean velocityMode) {
        this.velocityMode = velocityMode;
    }

    public void periodic() {
        if (!velocityMode) {
            double out = 0;
            if (shooting) {
                if (shooterTimer.get() < .125) {
                    out = -12;
                } else {
                    openLoopSetpoint = 0.0;
                    shooting = false;
                }
            } else if (enabled) {
                out = openLoopSetpoint;
            }
            openLoopSetpoint = out;
            applyOpenLoop();
        } else {
            // handled by command
//            applyVelocity();
        }
    }

    public void applyOpenLoop() {
        motor.setControl(openLoopOut.withOutput(openLoopSetpoint));
    }

    public void applyVelocity() {
        motor.setControl(velocityVoltage.withVelocity(this.velocityControl.velocitySetpoint));
    }

    public void shoot() {
        velocityMode = false;
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

    /**
     * Handles the feed note command logic
     */
    public void receiveFromIntake() {
        // drop out of shooting mode
        shooting = false;
        this.enableBreakMode();

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

    public void receiveFromIntakeVelocity() {
        // drop out of shooting mode
        velocityMode = true;
        shooting = false;
        this.enableBreakMode();

        // while we have not tripped out sensor
        if (!isNoteDetected()) {
            this.enabled = true;
            if (!isSlowSensorDetected()) {
                this.velocityControl.velocitySetpoint = slowFeedMode ? this.velocityControl.slowFeedVelocity : this.velocityControl.feedVelocity;
            } else {
                // Switch to slow speed mode when we see the note
                slowFeedMode = true;
                this.velocityControl.velocitySetpoint = this.velocityControl.slowFeedVelocity;
            }
        } else {
            // Set velocity
            this.velocityControl.velocitySetpoint = 0;
            this.openLoopSetpoint = 0.0;
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

    public void setSlowFeedShooterSpeed(double slowFeedShooterSpeed) {
        this.slowFeedShooterSpeed = slowFeedShooterSpeed;
    }

    public boolean isSlowFeedMode() {
        return slowFeedMode;
    }

    public void setSlowFeedMode(boolean slowFeedMode) {
        this.slowFeedMode = slowFeedMode;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty(name + "/Enabled", this::isEnabled, this::setEnabled);
        builder.addBooleanProperty(name + "/Note Detected", this::isNoteDetected, null);
        builder.addDoubleProperty(name + "/Open Loop Setpoint", this::getOpenLoopSetpoint, this::setOpenLoopSetpoint);
        builder.addBooleanProperty(name + "/Slow Sensor", this::isSlowSensorDetected, null);
        builder.addBooleanProperty(name + "/Slow Mode", this::isSlowFeedMode, this::setSlowFeedMode);

        if (Constants.Dashboard.ConfigurationMode && Constants.Dashboard.ShooterConfigMode) {
            builder.addDoubleProperty(name + "/Feed Note Speed", this::getFeedShooterSpeed, this::setFeedShooterSpeed);
            builder.addDoubleProperty(name + "/Shoot Speed", this::getShootingSpeed, this::setShootingSpeed);
            builder.addDoubleProperty(name + "/Slow Feed Speed", this::getSlowFeedShooterSpeed, this::setSlowFeedShooterSpeed);

            builder.addDoubleProperty(name + "/Velocity/Setpoint", () -> velocityControl.velocitySetpoint, (v) -> velocityControl.velocitySetpoint = v);
            builder.addDoubleProperty(name + "/Velocity/Slow Feed", () -> velocityControl.slowFeedVelocity, (v) -> velocityControl.slowFeedVelocity = v);
            builder.addDoubleProperty(name + "/Velocity/Feed", () -> velocityControl.feedVelocity, (v) -> velocityControl.feedVelocity = v);
        }
    }

    class VelocityControl {
        double velocitySetpoint = 0.0;
        double slowFeedVelocity = -2.0;
        double feedVelocity = -30.0;
    }
}