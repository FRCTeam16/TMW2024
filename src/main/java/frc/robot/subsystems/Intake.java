package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.util.CachedValue;
import frc.robot.subsystems.util.DoubleChanger;
import frc.robot.subsystems.util.PIDHelper;

import java.util.Objects;


public class Intake extends SubsystemBase implements Lifecycle, Sendable {

    private final TalonFX intakeDrive = new TalonFX(9); // google tells me the Krakens are on TalonFX for code
    private final TalonFX pivotDrive = new TalonFX(10); // this is geared down 48:1


    //
    // Intake
    //
    private final DutyCycleOut intakeDrive_Request = new DutyCycleOut(0.0); // CTRE pheonix 6 API
    private double intakeOpenLoopSeed = 0;
    private final IntakeSpeeds intakeSpeeds = new IntakeSpeeds();
    private final DigitalInput noteDetector = new DigitalInput(0);

    //
    // Pivot
    //
    private PIDHelper pidHelper = new PIDHelper("Pivot");
    private final TalonFXConfiguration pivotConfiguration = new TalonFXConfiguration();
    private final DutyCycleOut pivotOpenLoopDriveRequest = new DutyCycleOut(0);
    private final MotionMagicDutyCycle pivotMotionMagicDutyCycle = new MotionMagicDutyCycle(IntakePosition.Zero.setpoint);
    private boolean pivotOpenLoop = false;
    private double pivotOpenLoopSpeed = 0.0;
    private final PivotOpenLoopSpeeds pivotOpenLoopSpeeds = new PivotOpenLoopSpeeds();
    private double pivotSetpoint = 0;
    private IntakePosition pivotCurrentPosition = IntakePosition.Zero;

    //
    // Util
    //
    private final Telemetry telemetry;


    /**
     * Initializes the Intake subsystem.
     */
    public Intake() {
        intakeDrive.getConfigurator().apply(new TalonFXConfiguration());
        intakeDrive.setNeutralMode(NeutralModeValue.Coast);


//        pivotConfiguration.SoftwareLimitSwitch
//                .withForwardSoftLimitThreshold().withForwardSoftLimitEnable()
//                .withReverseSoftLimitThreshold().withReverseSoftLimitEnable();
//        pivotConfiguration.Slot0.withGravityType(GravityTypeValue.Arm_Cosine);
        pivotDrive.getConfigurator().apply(pivotConfiguration);

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

    public boolean isNoteDetected() {
        return !noteDetector.get();
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

    //
    // Pivot
    //


    public void setPivotOpenLoop(boolean pivotOpenLoop) {
        this.pivotOpenLoop = pivotOpenLoop;
    }

    public boolean isPivotOpenLoop() {
        return pivotOpenLoop;
    }


    public void goToPosition(IntakePosition newPosition) {
        pivotCurrentPosition = newPosition;
        telemetry.logPositionChange(newPosition);
    }

    public void setPivotSetpoint(double pivotSetpoint) {
        this.pivotSetpoint = pivotSetpoint;
    }

    public double getPivotSetpoint() {
        return this.pivotSetpoint;
    }

    public void holdPosition() {
        double pivotSetpoint = pivotDrive.getPosition().getValue();
    }

    public void pivotOpenLoopUp() {
        pivotOpenLoop = true;
        pivotOpenLoopSpeed = pivotOpenLoopSpeeds.getUpSpeed();
    }

    public void pivotOpenLoopDown() {
        pivotOpenLoop = true;
        pivotOpenLoopSpeed = pivotOpenLoopSpeeds.getDownSpeed();
    }

    @Override
    public void periodic() {
        double intakeOut = intakeOpenLoopSeed;
        if (isNoteDetected()) {
            intakeOut = 0.0;
        }
        intakeDrive.setControl(intakeDrive_Request.withOutput(intakeOut));

        if (pivotOpenLoop) {
            pivotDrive.setControl(pivotOpenLoopDriveRequest.withOutput(pivotOpenLoopSpeed));
        } else {
            if (pidHelper.updateValuesFromDashboard()) {
                pidHelper.updateConfiguration(pivotConfiguration.Slot0);
                pivotDrive.getConfigurator().apply(pivotConfiguration.Slot0);
            }
            pivotDrive.setControl(pivotMotionMagicDutyCycle.withPosition(pivotSetpoint));
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("IntakeSubsystem");

        builder.addDoubleProperty("Intake/FastSpeed", this.intakeSpeeds::getFastSpeed, this.intakeSpeeds::setFastSpeed);
        builder.addDoubleProperty("Intake/SlowSpeed", this.intakeSpeeds::getSlowSpeed, this.intakeSpeeds::setSlowSpeed);
        builder.addDoubleProperty("Intake/EjectSpeed", this.intakeSpeeds::getEjectSpeed, this.intakeSpeeds::setEjectSpeed);
        builder.addBooleanProperty("Intake/NoteDetected", this::isNoteDetected,null);

        builder.addBooleanProperty("Pivot/OpenLoop", this::isPivotOpenLoop, this::setPivotOpenLoop);
        builder.addDoubleProperty("Pivot/UpSpeed", pivotOpenLoopSpeeds::getUpSpeed, pivotOpenLoopSpeeds::setUpSpeed);
        builder.addDoubleProperty("Pivot/DownSpeed", pivotOpenLoopSpeeds::getDownSpeed, pivotOpenLoopSpeeds::setDownSpeed);
        builder.addDoubleProperty("Pivot/Setpoint", this::getPivotSetpoint, this::setPivotSetpoint);
    }

    public enum IntakePosition {
        Zero(0),
        Passing(30), // TODO: WILD GUESS
        Pickup(180), // TODO: WILD GUESS
        AMP(40);     // TODO: WILD GUESS

        private final int setpoint; // enum contructer lest us assign values (enums are technically classes)

        private IntakePosition(int setpoint) {
            this.setpoint = setpoint;
        }
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

    static class PivotOpenLoopSpeeds {
        private double upSpeed;
        private double downSpeed;

        public double getUpSpeed() {
            return upSpeed;
        }

        public void setUpSpeed(double upSpeed) {
            this.upSpeed = upSpeed;
        }

        public double getDownSpeed() {
            return downSpeed;
        }

        public void setDownSpeed(double downSpeed) {
            this.downSpeed = downSpeed;
        }
    }

    class Telemetry {
        private final StringLogEntry pivotPositionLog;
        private final DoubleLogEntry intakeSpeedLog;
        CachedValue<Double> intakeSpeedCache = new CachedValue<>(intakeOpenLoopSeed, DoubleChanger::areDifferent);
        CachedValue<IntakePosition> positionCache = new CachedValue<>(pivotCurrentPosition, Objects::equals);

        Telemetry() {
            DataLog log = DataLogManager.getLog();
            intakeSpeedLog = new DoubleLogEntry(log, "Intake/IntakeSpeed");
            pivotPositionLog = new StringLogEntry(log, "Intake/PivotPosition");
        }

        void logIntakeSpeed(double speed) {
            if (intakeSpeedCache.update(speed)) {
                intakeSpeedLog.append(speed);
            }
        }

        void logPositionChange(IntakePosition position) {
            if (positionCache.update(position)) {
                pivotPositionLog.append(position.name());
            }
        }

    }
}
