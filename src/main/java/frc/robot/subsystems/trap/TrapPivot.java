package frc.robot.subsystems.trap;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.util.*;

public class TrapPivot implements Lifecycle, Sendable {
    private final TalonFX motor;
    private final TalonFXConfiguration configuration = new TalonFXConfiguration();
    private final OpenLoopSpeedsConfig openLoopSpeeds = new OpenLoopSpeedsConfig(-1.2, 1.2);
    private final MotionMagicConfig motionMagicConfig = new MotionMagicConfig();
    private final SoftLimitValues softLimits = new SoftLimitValues(0, 0);

    private final VoltageOut openLoopOut = new VoltageOut(0);
    private final PositionVoltage positionOut = new PositionVoltage(0);
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
    private final PIDHelper pidHelper = new PIDHelper(Trap.SUBSYSTEM_NAME + "/Pivot/PID");
    private boolean openLoop = true;
    private boolean softLimitsEnabled = false;
    private double openLoopSetpoint = 0;
    private double closedLoopSetpoint;
    private TrapPivotPosition trapPosition = TrapPivotPosition.Zero;

    public TrapPivot(TalonFX motor) {
        this.motor = motor;
        motor.setNeutralMode(NeutralModeValue.Brake);
        motionMagicConfig.setAcceleration(150);
        motionMagicConfig.setVelocity(100);
        motionMagicConfig.setJerk(0);
        configuration
                .withSoftwareLimitSwitch(
                        new SoftwareLimitSwitchConfigs()
                                .withForwardSoftLimitThreshold(softLimits.forwardSoftLimitThreshold()).withForwardSoftLimitEnable(false)
                                .withReverseSoftLimitThreshold(softLimits.reverseSoftLimitThreshold()).withReverseSoftLimitEnable(true))
                .withHardwareLimitSwitch(
                        new HardwareLimitSwitchConfigs()
                                .withForwardLimitEnable(false)
                                .withReverseLimitEnable(false));


        pidHelper.initialize(0.81, 0.24, 0, 0, 0, 0);
        updatePIDFromDashboard();
        pidHelper.updateConfiguration(configuration.Slot0);
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
        this.motor.getConfigurator().apply(
                new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitThreshold(softLimits.forwardSoftLimitThreshold()).withForwardSoftLimitEnable(enable)
                        .withReverseSoftLimitThreshold(softLimits.reverseSoftLimitThreshold()).withReverseSoftLimitEnable(enable));
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
        return closedLoopSetpoint;
    }

    public void setClosedLoopSetpoint(double closedLoopSetpoint) {
        this.openLoop = false;
        this.closedLoopSetpoint = closedLoopSetpoint;
    }

    public void holdPosition() {
        this.setClosedLoopSetpoint(this.motor.getPosition().getValue());
    }

    public void setTrapPosition(TrapPivotPosition trapPosition) {
        this.trapPosition = trapPosition;
        this.setClosedLoopSetpoint(trapPosition.setpoint);
    }


    // TODO: make better ( aka don't hit things )
    // public void goToZero(){
    //     setTrapPosition(TrapPivotPosition.Zero);
    // }
    // public void goToFeed(){
    //     setTrapPosition(TrapPivotPosition.Feed);
    // }
    // public void goToDrive(){
    //     setTrapPosition(TrapPivotPosition.Drive);
    // }
    // public void goToTop(){
    //     setTrapPosition(TrapPivotPosition.Top);
    // }
    // public void goToScore(){
    //     setTrapPosition(TrapPivotPosition.Score);
    // }



    public void updatePIDFromDashboard() {
        if (pidHelper.updateValuesFromDashboard()) {
            BSLogger.log("TrapPivot", "updating pid");
            pidHelper.updateConfiguration(configuration.Slot0);
            motor.getConfigurator().apply(configuration.Slot0);
        }
        motionMagicConfig.updateSlot0Config(configuration.Slot0);
        motionMagicConfig.updateMotionMagicConfig(configuration.MotionMagic);
        StatusCode result = motor.getConfigurator().apply(configuration.MotionMagic);
//        if (result != StatusCode.OK) {
//            BSLogger.log("TrapPivot", "Failed to apply configuration to motor: " + result);
//        }
    }

    public Command updateClosedLoopFromDashbboardCommand() {
        return Commands.runOnce(this::updatePIDFromDashboard);
    }

    public void periodic() {
        updatePIDFromDashboard();
        if (openLoop) {
            motor.setControl(openLoopOut.withOutput(openLoopSetpoint));
        } else {
//            motor.setControl(positionOut.withPosition(closedLoopSetpoint));
            motor.setControl(motionMagicVoltage.withPosition(closedLoopSetpoint));
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("TrapPivot/Encoder", () -> this.motor.getPosition().getValue(), null);
        if (Constants.Dashboard.ConfigurationMode || Constants.Dashboard.TrapConfigMode) {

            builder.addBooleanProperty("TrapPivot/OpenLoop", this::isOpenLoop, this::setOpenLoop);
            builder.addDoubleProperty("TrapPivot/UpSpeed", openLoopSpeeds::getUpSpeed, openLoopSpeeds::setUpSpeed);
            builder.addDoubleProperty("TrapPivot/DownSpeed", openLoopSpeeds::getDownSpeed, openLoopSpeeds::setDownSpeed);

            builder.addDoubleProperty("TrapPivot/ClosedLoopSetpoint", this::getClosedLoopSetpoint, this::setClosedLoopSetpoint);
            
            builder.addDoubleProperty("TrapPivot/MM/kS", motionMagicConfig::getkS, motionMagicConfig::setkS);
            builder.addDoubleProperty("TrapPivot/MM/kG", motionMagicConfig::getkG, motionMagicConfig::setkG);
            builder.addDoubleProperty("TrapPivot/MM/Acceleration", motionMagicConfig::getAcceleration, motionMagicConfig::setAcceleration);
            builder.addDoubleProperty("TrapPivot/MM/Velocity", motionMagicConfig::getVelocity, motionMagicConfig::setVelocity);
            builder.addDoubleProperty("TrapPivot/MM/Jerk", motionMagicConfig::getJerk, motionMagicConfig::setJerk);

            builder.addBooleanProperty("TrapPivot/FwdLimitHit", () -> this.motor.getFault_ForwardSoftLimit().getValue(), null);
            builder.addBooleanProperty("TrapPivot/RevLimitHit", () -> this.motor.getFault_ReverseSoftLimit().getValue(), null);
        }
//        builder.addDoubleProperty("TrapPivot/ZeroEncoderOffset", this::getZeroPivotEncoderOffset, this::setZeroPivotEncoderOffset);
    }

    public enum TrapPivotPosition {
        Zero(0),
        Feed(21),
        Drive(20),
        Top(14),
        Score(8.8);


        private final double setpoint;

        TrapPivotPosition(double setpoint) {
            this.setpoint = setpoint;
        }
    }
}
