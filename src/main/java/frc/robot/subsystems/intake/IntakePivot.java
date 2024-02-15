package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.util.CachedValue;
import frc.robot.subsystems.util.PIDHelper;

import java.util.Objects;

public class IntakePivot implements Lifecycle, Sendable {
    private final TalonFX pivotDrive;
    private final PIDHelper pidHelper = new PIDHelper("IntakeSubsystem/Pivot/PID");
    private final TalonFXConfiguration pivotConfiguration = new TalonFXConfiguration();
    private final DutyCycleOut pivotOpenLoopDriveRequest = new DutyCycleOut(0);

    private final MotionMagicConfig motionMagicConfig = new MotionMagicConfig();
    private final MotionMagicDutyCycle pivotMotionMagicDutyCycle = new MotionMagicDutyCycle(IntakePosition.Zero.setpoint);
    private final PivotOpenLoopSpeeds pivotOpenLoopSpeeds = new PivotOpenLoopSpeeds();
    //
    // Util
    //
    private final Telemetry telemetry;
    private boolean pivotOpenLoop = false;
    private double pivotOpenLoopSpeed = 0.0;
    private double pivotSetpoint = 0;
    private IntakePosition pivotCurrentPosition = IntakePosition.Zero;

    public IntakePivot(TalonFX pivotDrive) {
        this.pivotDrive = pivotDrive;
        telemetry = new Telemetry();

        pidHelper.initialize(15, 0, 0, 0, 0.12, 0.01);

        pivotConfiguration.SoftwareLimitSwitch
                .withForwardSoftLimitThreshold(0).withForwardSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(-22).withReverseSoftLimitEnable(true);

        pidHelper.updateConfiguration(pivotConfiguration.Slot0);
        pivotConfiguration.Slot0.withGravityType(GravityTypeValue.Arm_Cosine);
        motionMagicConfig.updateMotionMagicConfig(pivotConfiguration.MotionMagic);

        DataLogManager.log("[Pivot:IntakePivot config: " + pivotConfiguration);
        pivotDrive.getConfigurator().apply(pivotConfiguration);

    }

    @Override
    public void teleopInit() {
        holdPosition();
    }

    @Override
    public void autoInit() {
        holdPosition();
    }

    public IntakePosition getIntakePosition() {
        return pivotCurrentPosition;
    }

    public void setIntakePosition(IntakePosition newPosition) {
        pivotCurrentPosition = newPosition;
        telemetry.logPositionChange(newPosition);
    }

    public double getPivotSetpoint() {
        return this.pivotSetpoint;
    }

    public void setPivotSetpoint(double pivotSetpoint) {
        setPivotOpenLoop(false);
        this.pivotSetpoint = pivotSetpoint;
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

    public boolean isPivotOpenLoop() {
        return pivotOpenLoop;
    }

    public void setPivotOpenLoop(boolean pivotOpenLoop) {
        this.pivotOpenLoop = pivotOpenLoop;
    }

    public void periodic() {
        if (pivotOpenLoop) {
            pivotDrive.setControl(pivotOpenLoopDriveRequest.withOutput(pivotOpenLoopSpeed));
            SmartDashboard.putNumber("PivotSubsystem/Pivot/RotationV", this.pivotDrive.getVelocity().getValueAsDouble());
            SmartDashboard.putNumber("PivotSubsystem/Pivot/RotationA", this.pivotDrive.getAcceleration().getValueAsDouble());
        } else {
//            if (pidHelper.updateValuesFromDashboard()) {
//                pidHelper.updateConfiguration(pivotConfiguration.Slot0);
//                pivotDrive.getConfigurator().apply(pivotConfiguration.Slot0);
//            }
            pivotDrive.setControl(pivotMotionMagicDutyCycle.withPosition(pivotSetpoint));
        }
    }

    public void updatePIDFromDashboard() {
        pidHelper.updateConfiguration(pivotConfiguration.Slot0);
        motionMagicConfig.updateMotionMagicConfig(pivotConfiguration.MotionMagic);
        pivotDrive.getConfigurator().apply(pivotConfiguration.Slot0);
    }

    public Command updatePIDFromDashbboardCommand() {
        return Commands.runOnce(this::updatePIDFromDashboard);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Pivot/OpenLoop", this::isPivotOpenLoop, this::setPivotOpenLoop);
        builder.addDoubleProperty("Pivot/UpSpeed", pivotOpenLoopSpeeds::getUpSpeed, pivotOpenLoopSpeeds::setUpSpeed);
        builder.addDoubleProperty("Pivot/DownSpeed", pivotOpenLoopSpeeds::getDownSpeed, pivotOpenLoopSpeeds::setDownSpeed);

        builder.addDoubleProperty("Pivot/Setpoint", this::getPivotSetpoint, this::setPivotSetpoint);
        builder.addStringProperty("Pivot/Position", () -> this.getIntakePosition().name(), null);

        builder.addDoubleProperty("Pivot/MM/kV", motionMagicConfig::getkV, motionMagicConfig::setkV);
        builder.addDoubleProperty("Pivot/MM/kA", motionMagicConfig::getkA, motionMagicConfig::setkA);
        builder.addDoubleProperty("Pivot/MM/Acceleration", motionMagicConfig::getAcceleration, motionMagicConfig::setAcceleration);
        builder.addDoubleProperty("Pivot/MM/Velocity", motionMagicConfig::getVelocity, motionMagicConfig::setVelocity);
        builder.addDoubleProperty("Pivot/MM/Jerk", motionMagicConfig::getJerk, motionMagicConfig::setJerk);


        builder.addBooleanProperty("Pivot/FwdLimitHit", () -> this.pivotDrive.getFault_ForwardSoftLimit().getValue(), null);
        builder.addBooleanProperty("Pivot/RevLimitHit", () -> this.pivotDrive.getFault_ReverseSoftLimit().getValue(), null);

        builder.addDoubleProperty("Pivot/Encoder", () -> this.pivotDrive.getPosition().getValue(), null);
    }


    public enum IntakePosition {
        Zero(0),
        Passing(0), // TODO: WILD GUESS
        Pickup(-20), // TODO: WILD GUESS
        AMP(-7);     // TODO: WILD GUESS

        private final int setpoint; // enum contructer lest us assign values (enums are technically classes)

        IntakePosition(int setpoint) {
            this.setpoint = setpoint;
        }
    }

    /**
     * Represents the open-loop speeds for intake pivot movement.
     */
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

    static class MotionMagicConfig {
        private double kA = 0.01;
        private double kV = 0.25;
        private double acceleration = 300;
        private double velocity = 150;
        private double jerk = 2000;

        public double getkA() {
            return kA;
        }

        public void setkA(double kA) {
            this.kA = kA;
        }

        public double getkV() {
            return kV;
        }

        public void setkV(double kV) {
            this.kV = kV;
        }

        public double getAcceleration() {
            return acceleration;
        }

        public void setAcceleration(double acceleration) {
            this.acceleration = acceleration;
        }

        public double getVelocity() {
            return velocity;
        }

        public void setVelocity(double velocity) {
            this.velocity = velocity;
        }

        public double getJerk() {
            return jerk;
        }

        public void setJerk(double jerk) {
            this.jerk = jerk;
        }

        public void updateMotionMagicConfig(MotionMagicConfigs config) {
            config.withMotionMagicExpo_kA(this.kA)
                    .withMotionMagicExpo_kV(this.kV)
                    .withMotionMagicAcceleration(this.acceleration)
                    .withMotionMagicCruiseVelocity(this.velocity)
                    .withMotionMagicJerk(this.jerk);
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
}
