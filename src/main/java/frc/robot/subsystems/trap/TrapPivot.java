package frc.robot.subsystems.trap;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.util.OpenLoopSpeedsConfig;
import frc.robot.subsystems.util.PIDHelper;
import frc.robot.subsystems.util.SoftLimitValues;

public class TrapPivot implements Lifecycle, Sendable {
    private final TalonFX motor;
    private final TalonFXConfiguration configuration = new TalonFXConfiguration();
    private final OpenLoopSpeedsConfig openLoopSpeeds = new OpenLoopSpeedsConfig(-1.2, 1.2);
    private final SoftLimitValues softLimits = new SoftLimitValues(30, -1);

    private final VoltageOut voltageOut = new VoltageOut(0);
    private final PIDHelper pidHelper = new PIDHelper(Trap.SUBSYSTEM_NAME + "/Pivot/PID");
    //
    // Encoder handling
    //
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(5);
    private final double zeroEncoderOffsetPosition = 0.881;
    private final PIDController pid;
    private boolean openLoop = true;
    private boolean softLimitsEnabled = false;
    private double openLoopSetpoint = 0;
    private TrapPivotPosition trapPosition = TrapPivotPosition.Zero;
    private TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);

    public TrapPivot(TalonFX motor) {
        this.motor = motor;
        motor.setNeutralMode(NeutralModeValue.Brake);
        configuration
                .withSoftwareLimitSwitch(
                        new SoftwareLimitSwitchConfigs()
                                .withForwardSoftLimitThreshold(softLimits.forwardSoftLimitThreshold()).withForwardSoftLimitEnable(false)
                                .withReverseSoftLimitThreshold(softLimits.reverseSoftLimitThreshold()).withReverseSoftLimitEnable(false))
                .withHardwareLimitSwitch(
                        new HardwareLimitSwitchConfigs()
                                .withForwardLimitEnable(false)
                                .withReverseLimitEnable(false));


//        pidHelper.initialize(0.81, 0.24, 0, 0, 0, 0);
        pidHelper.initialize(30, 0, 0, 0, 0, 0);

        pid = new PIDController(0, 0, 0);
        pidHelper.updatePIDController(pid);
        encoder.setPositionOffset(zeroEncoderOffsetPosition);
        motor.getConfigurator().apply(configuration);
    }

    @Override
    public void teleopInit() {
        holdPosition();
    }

    @Override
    public void autoInit() {
        holdPosition();
    }


    private void setSoftLimits(boolean enable) {
//        this.motor.getConfigurator().apply(
//                new SoftwareLimitSwitchConfigs()
//                        .withForwardSoftLimitThreshold(softLimits.forwardSoftLimitThreshold()).withForwardSoftLimitEnable(enable)
//                        .withReverseSoftLimitThreshold(softLimits.reverseSoftLimitThreshold()).withReverseSoftLimitEnable(enable));
    }


    public boolean isOpenLoop() {
        return openLoop;
    }

    public void setOpenLoop(boolean openLoop) {
        this.openLoop = openLoop;
    }

    public void openLoopUp() {
        setOpenLoop(true);
        setOpenLoopSetpoint(openLoopSpeeds.getUpSpeed());
    }

    public void openLoopDown() {
        setOpenLoop(true);
        setOpenLoopSetpoint(openLoopSpeeds.getDownSpeed());
    }

    public void stopOpenLoop() {
        // TODO: Enable once soft limit values are set
//        if (!softLimitsEnabled) {
//            softLimitsEnabled = true;
//            setSoftLimits(true);
//        }
        holdPosition();
    }

    public void unsafeOpenLoopUp() {
        softLimitsEnabled = false;
        setSoftLimits(false);
        openLoopUp();
    }

    public void unsafeOpenLoopDown() {
        softLimitsEnabled = false;
        setSoftLimits(false);
        openLoopDown();
    }

    public double getOpenLoopSetpoint() {
        return openLoopSetpoint;
    }

    public void setOpenLoopSetpoint(double openLoopSetpoint) {
        this.openLoopSetpoint = openLoopSetpoint;
    }

    public double getClosedLoopSetpoint() {
        return goal.position;
    }

    public void setClosedLoopSetpoint(double closedLoopSetpoint) {
        this.openLoop = false;
        this.goal = new TrapezoidProfile.State(closedLoopSetpoint, 0);
    }

    public void holdPosition() {
        this.setClosedLoopSetpoint(this.encoder.get());
    }

    public void setTrapPosition(TrapPivotPosition trapPosition) {
        this.trapPosition = trapPosition;
        this.setClosedLoopSetpoint(trapPosition.setpoint);
    }

    public void periodic() {
        if (openLoop) {
            motor.setControl(voltageOut.withOutput(openLoopSetpoint));
        } else {
            if (pidHelper.updateValuesFromDashboard()) {
                pidHelper.updatePIDController(pid);
            }
            final double output = pid.calculate(this.encoder.get(), goal.position);
            motor.setControl(voltageOut.withOutput(output));
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("TrapPivot/MotorPosition", () -> this.motor.getPosition().getValue(), null);
        builder.addDoubleProperty("TrapPivot/Encoder", this.encoder::get, null);
        if (Constants.Dashboard.ConfigurationMode || Constants.Dashboard.TrapConfigMode) {
            builder.addDoubleProperty("TrapPivot/EncoderAbs", this.encoder::getAbsolutePosition, null);
            builder.addBooleanProperty("TrapPivot/OpenLoop", this::isOpenLoop, this::setOpenLoop);
            builder.addDoubleProperty("TrapPivot/UpSpeed", openLoopSpeeds::getUpSpeed, openLoopSpeeds::setUpSpeed);
            builder.addDoubleProperty("TrapPivot/DownSpeed", openLoopSpeeds::getDownSpeed, openLoopSpeeds::setDownSpeed);

            builder.addDoubleProperty("TrapPivot/ClosedLoopSetpoint", this::getClosedLoopSetpoint, this::setClosedLoopSetpoint);

            builder.addBooleanProperty("TrapPivot/FwdLimitHit", () -> this.motor.getFault_ForwardSoftLimit().getValue(), null);
            builder.addBooleanProperty("TrapPivot/RevLimitHit", () -> this.motor.getFault_ReverseSoftLimit().getValue(), null);
        }
//        builder.addDoubleProperty("TrapPivot/ZeroEncoderOffset", this::getZeroPivotEncoderOffset, this::setZeroPivotEncoderOffset);
    }

    public enum TrapPivotPosition {

        Zero(0),
        Drive(0),

        AmpShot(0.211),
        AmpDeflect(0.35),

        PreClimb(0.18),
        Top(0.75),
        Score(0.75);


        private final double setpoint;

        TrapPivotPosition(double setpoint) {
            this.setpoint = setpoint;
        }
    }

}
