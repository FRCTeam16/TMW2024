package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.util.CachedValue;
import frc.robot.subsystems.util.DoubleChanger;

public class IntakeSpeed implements Lifecycle, Sendable {
    private final TalonFX intakeDrive;

    private final VoltageOut intakeDrive_Request = new VoltageOut(0.0);

    private final IntakeSpeedVolts intakeSpeedVolts = new IntakeSpeedVolts();

    private double intakeOpenLoopSpeed = 0;


    public IntakeSpeed(TalonFX intakeDrive) {
        this.intakeDrive = intakeDrive;
        intakeDrive.getConfigurator().apply(new TalonFXConfiguration());
        intakeDrive.setNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public void teleopInit() {
        intakeOpenLoopSpeed = 0.0;
    }

    @Override
    public void autoInit() {
        intakeOpenLoopSpeed = 0.0;
    }

    public void runIntakeFast() {
        intakeOpenLoopSpeed = this.intakeSpeedVolts.getFastSpeed();
    }

    public void runIntakeFeedShooter() {
        intakeOpenLoopSpeed = this.intakeSpeedVolts.getFeedShooterIntakeSpeed();
    }

    public void runIntakeEject() {
        intakeOpenLoopSpeed = this.intakeSpeedVolts.getEjectSpeed();
    }

    public void runAmpShot() {
        intakeOpenLoopSpeed = this.intakeSpeedVolts.getAmpShotSpeed();
    }

    public void stopIntake() {
        intakeOpenLoopSpeed = 0.0;
    }

    public void runIntakeDebug(double value) {
        intakeOpenLoopSpeed = value;
    }

    void periodic() {
        intakeDrive.setControl(intakeDrive_Request.withOutput(intakeOpenLoopSpeed));
    }



    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Intake/IntakeSpeed", () -> this.intakeOpenLoopSpeed, null);
        if (Constants.Dashboard.ConfigurationMode) {
            builder.addDoubleProperty("Intake/FastSpeed", this.intakeSpeedVolts::getFastSpeed, this.intakeSpeedVolts::setFastSpeed);
            builder.addDoubleProperty("Intake/FeedNote", this.intakeSpeedVolts::getFeedShooterIntakeSpeed, this.intakeSpeedVolts::setFeedShooterIntakeSpeed);
            builder.addDoubleProperty("Intake/EjectSpeed", this.intakeSpeedVolts::getEjectSpeed, this.intakeSpeedVolts::setEjectSpeed);
            builder.addDoubleProperty("Intake/AmpShootSpeed", this.intakeSpeedVolts::getAmpShotSpeed, this.intakeSpeedVolts::setAmpShotSpeed);
            builder.addDoubleProperty("Intake/CenterSpeedIntake", this.intakeSpeedVolts::getCenterSpeedIntake, this.intakeSpeedVolts::setCenterSpeedIntake);
            builder.addDoubleProperty("Intake/CenterSpeedEject", this.intakeSpeedVolts::getCenterSpeedEject, this.intakeSpeedVolts::setCenterSpeedEject);
        }
    }

    public void runCenterSpeedEject() {
        intakeOpenLoopSpeed = this.intakeSpeedVolts.getCenterSpeedEject();
    }

    public void runCenterSpeedIntake() {
        intakeOpenLoopSpeed = this.intakeSpeedVolts.getCenterSpeedIntake();
    }

    static class IntakeSpeedVolts {
        private double fastSpeed = 6.0;
//        private double slowSpeed = -2;
        private double ejectSpeed = -6;
        private double centerSpeedIntake = 4.8;
        private double centerSpeedEject = -6.0;
        private double feedShooterIntakeSpeed = -3;
        private double ampShotSpeed = -8.00;

        // Getter for fastSpeed
        public double getFastSpeed() {
            return fastSpeed;
        }

        // Setter for fastSpeed
        public void setFastSpeed(double fastSpeed) {
            this.fastSpeed = fastSpeed;
        }

        // Getter for ejectSpeed
        public double getEjectSpeed() {
            return ejectSpeed;
        }

        // Setter for ejectSpeed
        public void setEjectSpeed(double ejectSpeed) {
            this.ejectSpeed = ejectSpeed;
        }

        // Getter for feedShooterIntakeSpeed
        public double getFeedShooterIntakeSpeed() {
            return feedShooterIntakeSpeed;
        }

        // Setter for feedShooterIntakeSpeed
        public void setFeedShooterIntakeSpeed(double feedShooterIntakeSpeed) {
            this.feedShooterIntakeSpeed = feedShooterIntakeSpeed;
        }

        // Getter for ampShotSpeed
        public double getAmpShotSpeed() {
            return ampShotSpeed;
        }

        // Setter for ampShotSpeed
        public void setAmpShotSpeed(double ampShotSpeed) {
            this.ampShotSpeed = ampShotSpeed;
        }

        public double getCenterSpeedEject() {
            return centerSpeedEject;
        }

        public void setCenterSpeedEject(double centerSpeedEject) {
            this.centerSpeedEject = centerSpeedEject;
        }

        public double getCenterSpeedIntake() {
            return centerSpeedIntake;
        }

        public void setCenterSpeedIntake(double centerSpeedIntake) {
            this.centerSpeedIntake = centerSpeedIntake;
        }
    }



    class Telemetry {
        private final DoubleLogEntry intakeSpeedLog;
        CachedValue<Double> intakeSpeedCache = new CachedValue<>(intakeOpenLoopSpeed, DoubleChanger::areDifferent);

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
