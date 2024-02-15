package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.util.CachedValue;
import frc.robot.subsystems.util.PIDHelper;

import java.util.Objects;

public class IntakePivot implements Lifecycle, Sendable {
    private final TalonFX pivotDrive;
    private final PIDHelper pidHelper = new PIDHelper("IntakePivot");
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

    public enum IntakePosition {
        Zero(0),
        Passing(30), // TODO: WILD GUESS
        Pickup(180), // TODO: WILD GUESS
        AMP(40);     // TODO: WILD GUESS

        private final int setpoint; // enum contructer lest us assign values (enums are technically classes)

        IntakePosition(int setpoint) {
            this.setpoint = setpoint;
        }
    }

    public IntakePivot(TalonFX pivotDrive) {
        this.pivotDrive = pivotDrive;
        pivotDrive.getConfigurator().apply(pivotConfiguration);
        telemetry = new Telemetry();

        //        pivotConfiguration.SoftwareLimitSwitch
//                .withForwardSoftLimitThreshold().withForwardSoftLimitEnable()
//                .withReverseSoftLimitThreshold().withReverseSoftLimitEnable();
//        pivotConfiguration.Slot0.withGravityType(GravityTypeValue.Arm_Cosine);

    }

    @Override
    public void teleopInit() {
        Lifecycle.super.teleopInit();
    }

    @Override
    public void autoInit() {
        Lifecycle.super.autoInit();
    }

    public void setIntakePosition(IntakePosition newPosition) {
        pivotCurrentPosition = newPosition;
        telemetry.logPositionChange(newPosition);
    }

    public IntakePosition getIntakePosition() {
        return pivotCurrentPosition;
    }

    public void setPivotSetpoint(double pivotSetpoint) {
        setPivotOpenLoop(false);
        this.pivotSetpoint = pivotSetpoint;
    }

    public double getPivotSetpoint() {
        return this.pivotSetpoint;
    }

    public void holdPosition() {
        double pivotSetpoint = pivotDrive.getPosition().getValue();
        setPivotSetpoint(pivotSetpoint);
    }

    public void pivotOpenLoopUp() {
        pivotOpenLoop = true;
        pivotOpenLoopSpeed = pivotOpenLoopSpeeds.getUpSpeed();
    }

    public void pivotOpenLoopDown() {
        pivotOpenLoop = true;
        pivotOpenLoopSpeed = pivotOpenLoopSpeeds.getDownSpeed();
    }

    public void pivotOpenLoopStop() {
        pivotOpenLoop = true;
        pivotOpenLoopSpeed = 0.0;
    }

    public void setPivotOpenLoop(boolean pivotOpenLoop) {
        this.pivotOpenLoop = pivotOpenLoop;
    }

    public boolean isPivotOpenLoop() {
        return pivotOpenLoop;
    }



    public void periodic() {
        if (pivotOpenLoop) {
            pivotDrive.setControl(pivotOpenLoopDriveRequest.withOutput(pivotOpenLoopSpeed));
            SmartDashboard.putNumber("PivotSubsystem/RotationV", this.pivotDrive.getVelocity().getValueAsDouble());
            SmartDashboard.putNumber("PivotSubsystem/RotationA", this.pivotDrive.getAcceleration().getValueAsDouble());
        } else {
            if (pidHelper.updateValuesFromDashboard()) {
                pidHelper.updateConfiguration(pivotConfiguration.Slot0);
                pivotDrive.getConfigurator().apply(pivotConfiguration.Slot0);
            }
            pivotDrive.setControl(pivotMotionMagicDutyCycle.withPosition(pivotSetpoint));
        }
    }


    static class PivotOpenLoopSpeeds {
        private double upSpeed = .5;
        private double downSpeed = -.5;

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
        CachedValue<IntakePosition> positionCache = new CachedValue<>(pivotCurrentPosition, Objects::equals);

        Telemetry() {
            DataLog log = DataLogManager.getLog();
            pivotPositionLog = new StringLogEntry(log, "Intake/PivotPosition");
        }

        void logPositionChange(IntakePosition position) {
            if (positionCache.update(position)) {
                pivotPositionLog.append(position.name());
            }
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Pivot/OpenLoop", this::isPivotOpenLoop, this::setPivotOpenLoop);
        builder.addDoubleProperty("Pivot/UpSpeed", pivotOpenLoopSpeeds::getUpSpeed, pivotOpenLoopSpeeds::setUpSpeed);
        builder.addDoubleProperty("Pivot/DownSpeed", pivotOpenLoopSpeeds::getDownSpeed, pivotOpenLoopSpeeds::setDownSpeed);
        builder.addDoubleProperty("Pivot/Setpoint", this::getPivotSetpoint, this::setPivotSetpoint);
        builder.addStringProperty("Pivot/Position", () -> this.getIntakePosition().name(), null);

        builder.addDoubleProperty("Pivot/Encoder", () -> this.pivotDrive.getPosition().getValue(), null);
    }
}
