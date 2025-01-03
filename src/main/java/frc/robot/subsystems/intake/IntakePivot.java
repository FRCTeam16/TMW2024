package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.util.*;

import java.util.Objects;

public class IntakePivot implements Lifecycle, Sendable {
    private final TalonFX pivotDrive;
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(4);
    private final PIDHelper pidHelper = new PIDHelper("IntakeSubsystem/Pivot/PID");
    private final TalonFXConfiguration pivotConfiguration = new TalonFXConfiguration();
    private final VoltageOut voltageOut = new VoltageOut(0);

    private final MotionMagicConfig motionMagicConfig = new MotionMagicConfig();
    private final MotionMagicVoltage pivotMotionMagicDutyCycle = new MotionMagicVoltage(IntakePosition.Zero.setpoint);
    private final OpenLoopSpeedsConfig pivotOpenLoopSpeeds = new OpenLoopSpeedsConfig(0.083, -0.083);

    private final AmpShotController ampShotPIDController = new AmpShotController(encoder, 0.145);


    //
    // Util
    //
    private final Telemetry telemetry;
    // the amount to offset the absolute encoder to make it show zero when we are in a Zero position
//    private final double zeroEncoderOffsetPosition = 0.9343;
    private final double encoderToMotorRatio = -48;
    private boolean softLimitsEnabled = true;
    private boolean pivotOpenLoop = false;
    private double pivotOpenLoopSpeed = 0.0;
    private double pivotSetpoint = 0;
    private IntakePosition pivotCurrentPosition = IntakePosition.Zero;


    private static final double MAXIMUM_LIMIT = 1;
    private static final double MINIMUM_LIMIT = -25;
    private boolean encoderReset = false;   // flag to only allow call-once semantics for state initialization
    private boolean encoderControl = false; // whether we are in fine grained encoder control

    public IntakePivot(TalonFX pivotDrive) {
        this.pivotDrive = pivotDrive;
        telemetry = new Telemetry();

        encoder.reset();

        // kp = 0.25, ka = 0.01
        // kp 15, ka = 0.01
//        pidHelper.initialize(0.7, 0.2, 0, 0, 0.12, 0.01);
        pidHelper.initialize(4.0, 0.0, 0, 0, 0.12, 0.0);
        motionMagicConfig.setkS(0);
        motionMagicConfig.setkG(0);
        motionMagicConfig.setVelocity(50);
        motionMagicConfig.setAcceleration(250);
        motionMagicConfig.setJerk(500);

        pidHelper.updateConfiguration(pivotConfiguration.Slot0);
        pivotConfiguration.Slot0.withGravityType(GravityTypeValue.Arm_Cosine);

        motionMagicConfig.updateSlot0Config(pivotConfiguration.Slot0);
        motionMagicConfig.updateMotionMagicConfig(pivotConfiguration.MotionMagic);

        pivotConfiguration
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withSupplyCurrentLimit(30)
                                .withSupplyCurrentLimitEnable(true)
                ).withSoftwareLimitSwitch(
                        new SoftwareLimitSwitchConfigs()
                                .withForwardSoftLimitThreshold(MAXIMUM_LIMIT).withForwardSoftLimitEnable(softLimitsEnabled)
                                .withReverseSoftLimitThreshold(MINIMUM_LIMIT).withReverseSoftLimitEnable(softLimitsEnabled)
                ).withHardwareLimitSwitch(
                        new HardwareLimitSwitchConfigs()
                                .withForwardLimitEnable(false)
                                .withReverseLimitEnable(false)
                );


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

    /**
     * If the Kraken reads an incorrect position, we will us the absolute encoder to fix the position
     */
    public void fixMotorPosition() {
        double realMotorPosition = encoder.get() * encoderToMotorRatio;
        BSLogger.log("IntakePivot", "fixMotorPosition to: " + realMotorPosition);
        pivotDrive.setPosition(realMotorPosition);
    }

    public Command fixMotorPositionCmd() {
        return Commands.runOnce(this::fixMotorPosition);
    }

    public IntakePosition getIntakePosition() {
        return pivotCurrentPosition;
    }

    public void setIntakePosition(IntakePosition newPosition) {
        if (IntakePosition.AMPShot == newPosition || IntakePosition.Climb == newPosition) {
            ampShotPIDController.setSetpoint(newPosition.setpoint);
        } else {
            setPivotSetpoint(newPosition.setpoint);
        }
        pivotCurrentPosition = newPosition;
        telemetry.logPositionChange(newPosition);
    }

    public double getPivotSetpoint() {
        return this.pivotSetpoint;
    }

    public void setPivotSetpoint(double pivotSetpoint) {
        if (pivotSetpoint < MINIMUM_LIMIT || pivotSetpoint > MAXIMUM_LIMIT) {
            BSLogger.log("IntakePivot", "Ignoring pivot setpoint request due to range violation, asked for: " + pivotSetpoint);
            return;
        }
        setPivotOpenLoop(false);
        this.pivotSetpoint = pivotSetpoint;
    }

    public void holdPosition() {
        double pivotSetpoint = pivotDrive.getPosition().getValue();
        setPivotSetpoint(pivotSetpoint);
    }

    public double getCurrentPosition() {
        return pivotDrive.getPosition().getValue();
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
            pivotDrive.setControl(voltageOut.withOutput(pivotOpenLoopSpeed)
                    .withLimitForwardMotion(softLimitsEnabled)
                    .withLimitReverseMotion(softLimitsEnabled));
            SmartDashboard.putNumber(Intake.SUBSYSTEM_NAME + "/Pivot/RotationV", this.pivotDrive.getVelocity().getValueAsDouble());
            SmartDashboard.putNumber(Intake.SUBSYSTEM_NAME + "/Pivot/RotationA", this.pivotDrive.getAcceleration().getValueAsDouble());
        } else {
//            if (pidHelper.updateValuesFromDashboard()) {
//                pidHelper.updateConfiguration(pivotConfiguration.Slot0);
//                pivotDrive.getConfigurator().apply(pivotConfiguration.Slot0);
//            }

            if (IntakePosition.MotorPreAMPShot == pivotCurrentPosition) {
                double grossThreshold = 0.25;
                double currentPosition= pivotDrive.getPosition().getValue();
                if ((currentPosition > IntakePosition.MotorPreAMPShot.setpoint - grossThreshold) &&
                (currentPosition < IntakePosition.MotorPreAMPShot.setpoint + grossThreshold)) {
                    BSLogger.log("IntakePivot", "Flipping to fine grained control");
                    setIntakePosition(IntakePosition.AMPShot);
                }
            }
            if (IntakePosition.AMPShot == pivotCurrentPosition || IntakePosition.Climb == pivotCurrentPosition) {
                encoderControl = true;
                pivotDrive.setControl(voltageOut.withOutput(ampShotPIDController.calculate()));
            } else {
                encoderControl = false;
                pivotDrive.setControl(pivotMotionMagicDutyCycle.withPosition(pivotSetpoint)
                        .withLimitForwardMotion(softLimitsEnabled)
                        .withLimitReverseMotion(softLimitsEnabled));
            }
        }
    }

   public MotionMagicConfigs getMotionMagicConfigs() {
       return pivotConfiguration.MotionMagic;
   }

   public void applyConfigs(MotionMagicConfigs cfg) {
        pivotDrive.getConfigurator().apply(cfg);
   }

    public void updatePIDFromDashboard() {
        pidHelper.updateConfiguration(pivotConfiguration.Slot0);
        motionMagicConfig.updateSlot0Config(pivotConfiguration.Slot0);
        motionMagicConfig.updateMotionMagicConfig(pivotConfiguration.MotionMagic);
        StatusCode result = pivotDrive.getConfigurator().apply(pivotConfiguration);
        BSLogger.log("IntakePivot", "updatePIDFromDashboard result: " + result);
    }

    public Command updatePIDFromDashbboardCommand() {
        return Commands.runOnce(this::updatePIDFromDashboard).ignoringDisable(true);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Pivot/Setpoint", this::getPivotSetpoint, this::setPivotSetpoint);
        builder.addStringProperty("Pivot/PositionName", () -> this.getIntakePosition().name(), null);
        builder.addDoubleProperty("Pivot/MotorPosition", () -> this.pivotDrive.getPosition().getValue(), null);
        builder.addDoubleProperty("Pivot/EncoderPosition", this.encoder::get, null);
        builder.addDoubleProperty("Pivot/EncoderAbsolutePosition", this.encoder::getAbsolutePosition, null);
        builder.addDoubleProperty("Pivot/EncoderOffset", this.encoder::getPositionOffset, null);
        builder.addBooleanProperty("Pivot/EncoderControl", () -> this.encoderControl, null);

        
        if (Constants.Dashboard.ConfigurationMode && Constants.Dashboard.IntakeConfigMode) {
            ampShotPIDController.initSendable(builder);

            builder.addBooleanProperty("Pivot/OpenLoop", this::isPivotOpenLoop, this::setPivotOpenLoop);
            builder.addDoubleProperty("Pivot/UpSpeed", pivotOpenLoopSpeeds::getUpSpeed, pivotOpenLoopSpeeds::setUpSpeed);
            builder.addDoubleProperty("Pivot/DownSpeed", pivotOpenLoopSpeeds::getDownSpeed, pivotOpenLoopSpeeds::setDownSpeed);

            builder.addDoubleProperty("Pivot/MM/kS", motionMagicConfig::getkS, motionMagicConfig::setkS);
            builder.addDoubleProperty("Pivot/MM/kG", motionMagicConfig::getkG, motionMagicConfig::setkG);
            builder.addDoubleProperty("Pivot/MM/Acceleration", motionMagicConfig::getAcceleration, motionMagicConfig::setAcceleration);
            builder.addDoubleProperty("Pivot/MM/Velocity", motionMagicConfig::getVelocity, motionMagicConfig::setVelocity);
            builder.addDoubleProperty("Pivot/MM/Jerk", motionMagicConfig::getJerk, motionMagicConfig::setJerk);

            builder.addBooleanProperty("Pivot/FwdLimitHit", () -> this.pivotDrive.getFault_ForwardSoftLimit().getValue(), null);
            builder.addBooleanProperty("Pivot/RevLimitHit", () -> this.pivotDrive.getFault_ReverseSoftLimit().getValue(), null);
        }
    }

    public boolean isInPosition(IntakePosition position) {
        return (Math.abs(getCurrentPosition() - position.setpoint) <= 0.15);
    }

    public boolean isMotorInPosition(IntakePosition position) {
        return (Math.abs(getCurrentPosition() - position.setpoint) <= 2.0);
    }

    void resetEncoder() {
        BSLogger.log("IntakePivot", "resetEncoder");
        this.encoder.reset();
    }

    public void resetEncoderOnce() {
        if (!encoderReset) {
            encoderReset = true;
            resetEncoder();
        }
    }


    public enum IntakePosition {
        Zero(0),
        Vertical(-9.833),
        Pickup(-24.6),
        MotorPreAMPShot(-9),
        AMPShot(0.190),
        Eject(-13),
        Climb(0.11);

        private final double setpoint;

        IntakePosition(double setpoint) {
            this.setpoint = setpoint;
        }
    }

    class Telemetry {
        private final StringLogEntry pivotPositionLog;
        CachedValue<IntakePosition> positionCache = new CachedValue<>(pivotCurrentPosition, Objects::equals);

        Telemetry() {
            DataLog log = DataLogManager.getLog();
            pivotPositionLog = new StringLogEntry(log, "datalog/Intake/PivotPosition");
        }

        void logPositionChange(IntakePosition position) {
            if (positionCache.update(position)) {
                pivotPositionLog.append(position.name());
            }
        }
    }
}
