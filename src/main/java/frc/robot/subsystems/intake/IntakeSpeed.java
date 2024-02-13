package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.util.CachedValue;
import frc.robot.subsystems.util.DoubleChanger;

public class IntakeSpeed implements Lifecycle, Sendable {
    private final TalonFX intakeDrive;

    private final DutyCycleOut intakeDrive_Request = new DutyCycleOut(0.0); // CTRE pheonix 6 API
    private final IntakeSpeeds intakeSpeeds = new IntakeSpeeds();
    private final Telemetry telemetry;
    private double intakeOpenLoopSeed = 0;


    public IntakeSpeed(TalonFX intakeDrive) {
        this.intakeDrive = intakeDrive;
        intakeDrive.getConfigurator().apply(new TalonFXConfiguration());
        intakeDrive.setNeutralMode(NeutralModeValue.Coast);

        telemetry = new Telemetry();
    }

    @Override
    public void teleopInit() {
        intakeOpenLoopSeed = 0.0;
    }

    @Override
    public void autoInit() {
        intakeOpenLoopSeed = 0.0;
    }

    public void runIntakeFast() {
        intakeOpenLoopSeed = this.intakeSpeeds.getFastSpeed();
        telemetry.logIntakeSpeed(intakeOpenLoopSeed);
    }

    public void runIntakeSlow() {
        intakeOpenLoopSeed = this.intakeSpeeds.getSlowSpeed();
        telemetry.logIntakeSpeed(intakeOpenLoopSeed);
    }

    public void runIntakeEject() {
        intakeOpenLoopSeed = this.intakeSpeeds.getEjectSpeed();
        telemetry.logIntakeSpeed(intakeOpenLoopSeed);
    }

    public void stopIntake() {
        intakeOpenLoopSeed = 0.0;
        telemetry.logIntakeSpeed(intakeOpenLoopSeed);
    }

    void periodic() {
        intakeDrive.setControl(intakeDrive_Request.withOutput(intakeOpenLoopSeed));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Intake/FastSpeed", this.intakeSpeeds::getFastSpeed, this.intakeSpeeds::setFastSpeed);
        builder.addDoubleProperty("Intake/SlowSpeed", this.intakeSpeeds::getSlowSpeed, this.intakeSpeeds::setSlowSpeed);
        builder.addDoubleProperty("Intake/EjectSpeed", this.intakeSpeeds::getEjectSpeed, this.intakeSpeeds::setEjectSpeed);
    }

    static class IntakeSpeeds {
        private double fastSpeed = 0.75;
        private double slowSpeed = 0.15;
        private double ejectSpeed = -0.50;

        public double getFastSpeed() {
            return fastSpeed;
        }

        public void setFastSpeed(double fastSpeed) {
            this.fastSpeed = fastSpeed;
        }

        public double getSlowSpeed() {
            return slowSpeed;
        }

        public void setSlowSpeed(double slowSpeed) {
            this.slowSpeed = slowSpeed;
        }

        public double getEjectSpeed() {
            return ejectSpeed;
        }

        public void setEjectSpeed(double ejectSpeed) {
            this.ejectSpeed = ejectSpeed;
        }
    }


    class Telemetry {
        private final DoubleLogEntry intakeSpeedLog;
        CachedValue<Double> intakeSpeedCache = new CachedValue<>(intakeOpenLoopSeed, DoubleChanger::areDifferent);

        Telemetry() {
            DataLog log = DataLogManager.getLog();
            intakeSpeedLog = new DoubleLogEntry(log, "Intake/IntakeSpeed");
        }

        void logIntakeSpeed(double speed) {
            if (intakeSpeedCache.update(speed)) {
                intakeSpeedLog.append(speed);
            }
        }
    }
}
