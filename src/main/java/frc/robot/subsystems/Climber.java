package frc.robot.subsystems;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.util.OpenLoopSpeedsConfig;
import frc.robot.subsystems.util.PIDHelper;
import frc.robot.subsystems.util.SoftLimitValues;

public class Climber extends SubsystemBase implements Lifecycle, Sendable {
    public static final String SUBSYSTEM_NAME = "ClimberSubsystem";
    // motor configuration
    private final TalonFX climberDrive = new TalonFX(35);// set to 125:1
    private final VoltageOut openLoopOut = new VoltageOut(0).withEnableFOC(true);
    private final PositionVoltage positionOut = new PositionVoltage(0);

    private final PIDHelper pidHelper = new PIDHelper(SUBSYSTEM_NAME + "/PID");
    private final OpenLoopSpeedsConfig climberOpenLoopSpeeds = new OpenLoopSpeedsConfig(6, -6);
    private final TalonFXConfiguration config;

    private final SoftLimitValues softLimits = new SoftLimitValues(0, -250);

    //
    // Open Loop Handling
    //

    // Whether the climber is in open loop mode
    private boolean openLoop = false;
    // Current open loop speed
    private double openLoopSpeed = 0.0;
    // Current climber position
    private ClimberPosition currentPosition = ClimberPosition.DOWN;
    // Closed loop setpoint
    private double setpoint = 0.0;
    private boolean softLimitsEnabled = true;

    public Climber() {
        pidHelper.initialize(2.0, 0, 0, 0, 0, 0);
        config = new TalonFXConfiguration()
                .withSoftwareLimitSwitch(
                        new SoftwareLimitSwitchConfigs()
                                .withForwardSoftLimitThreshold(softLimits.forwardSoftLimitThreshold())
                                .withForwardSoftLimitEnable(softLimitsEnabled)
                                .withReverseSoftLimitThreshold(softLimits.reverseSoftLimitThreshold())
                                .withReverseSoftLimitEnable(softLimitsEnabled))
                .withHardwareLimitSwitch(
                        new HardwareLimitSwitchConfigs()
                                .withForwardLimitEnable(false)
                                .withReverseLimitEnable(false));
        pidHelper.updateConfiguration(config.Slot0);
        climberDrive.getConfigurator().apply(config);
        climberDrive.setNeutralMode(NeutralModeValue.Brake);
        setpoint = climberDrive.getPosition().getValue();
    }

    @Override
    public void teleopInit() {
        setpoint = climberDrive.getPosition().getValue();
    }

    @Override
    public void autoInit() {
        setpoint = climberDrive.getPosition().getValue();

    }

    public boolean isOpenLoop() {
        return openLoop;
    }

    public void setOpenLoop(boolean openLoop) {
        this.openLoop = openLoop;
    }

    public void openLoopUp() {
        setOpenLoop(true);
        openLoopSpeed = climberOpenLoopSpeeds.getUpSpeed();
    }

    public void openLoopDown() {
        setOpenLoop(true);
        openLoopSpeed = climberOpenLoopSpeeds.getDownSpeed();
    }

    public void unsafeOpenLoopUp() {
        softLimitsEnabled = false;
        configureSoftLimits(false);
        openLoopUp();
    }

    public void unsafeOpenLoopDown() {
        softLimitsEnabled = false;
        configureSoftLimits(false);
        openLoopDown();
    }

    private void configureSoftLimits(boolean enable) {
        climberDrive.getConfigurator().apply(
                new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitThreshold(softLimits.forwardSoftLimitThreshold()).withForwardSoftLimitEnable(enable)
                        .withReverseSoftLimitThreshold(softLimits.reverseSoftLimitThreshold()).withReverseSoftLimitEnable(enable));
    }


    /**
     * Stop the climber from moving in open loop mode
     */
    public void holdPosition() {
        if (!softLimitsEnabled) {
            softLimitsEnabled = true;
            configureSoftLimits(true);
        }
        openLoop = true;
        openLoopSpeed = 0.0;
        setpoint = climberDrive.getPosition().getValue();
    }

    public Command moveToStateCmd(ClimberPosition climberPosition) {
        return this.runOnce(() -> setClimberPosition(climberPosition));
    }

    public ClimberPosition getClimberPosition() {
        return this.currentPosition;
    }

    public void setClimberPosition(ClimberPosition position) {
        this.currentPosition = position;
        this.setSetpoint(
                switch (position) {
                    case UP -> ClimberPositionValues.UP;
                    case DOWN -> ClimberPositionValues.DOWN;
                });
    }

    public double getSetpoint() {
        return setpoint;
    }

    private void setSetpoint(double setpoint) {
        setOpenLoop(false);
        this.setpoint = setpoint;
    }

    public void bumpSetpoint(double modifiedAmmt) {
        this.setpoint += modifiedAmmt;
    }

    @Override
    public void periodic() {
        if (Constants.Dashboard.ClimberConfigMode && Constants.Dashboard.ClimberConfigMode && pidHelper.updateValuesFromDashboard()) {
            pidHelper.updateConfiguration(config.Slot0);
            climberDrive.getConfigurator().apply(config.Slot0);
        }

        if (openLoop) {
            climberDrive.setControl(openLoopOut.withOutput(openLoopSpeed));
        } else {
            climberDrive.setControl(positionOut.withPosition(this.setpoint));
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        if (Constants.Dashboard.UseSendables) {
            builder.setSmartDashboardType(SUBSYSTEM_NAME);
            builder.addStringProperty("ClimberPosition", () -> this.getClimberPosition().name(), null);
            builder.addBooleanProperty("OpenLoopEnabled", this::isOpenLoop, this::setOpenLoop);
            builder.addDoubleProperty("Setpoint", this::getSetpoint, this::setSetpoint);
            builder.addDoubleProperty("Encoder", () -> this.climberDrive.getPosition().getValue(), null);
            builder.addDoubleProperty("Velocity", () -> this.climberDrive.getVelocity().getValue(), null);
            builder.addDoubleProperty("Acceleration", () -> this.climberDrive.getAcceleration().getValue(), null);
            builder.addDoubleProperty("OpenLoop/CurrentSpeed", () -> this.openLoopSpeed, null);
            builder.addBooleanProperty("SoftLimitsEnabled", () -> this.softLimitsEnabled, (value) -> {
                this.softLimitsEnabled = value;
                this.configureSoftLimits(value);
            });

            if (Constants.Dashboard.ConfigurationMode) {
                builder.addDoubleProperty("OpenLoop/UpSpeed", this.climberOpenLoopSpeeds::getUpSpeed, this.climberOpenLoopSpeeds::setUpSpeed);
                builder.addDoubleProperty("OpenLoop/DownSpeed", this.climberOpenLoopSpeeds::getDownSpeed, this.climberOpenLoopSpeeds::setDownSpeed);
                builder.addDoubleProperty("Positions/UP", () -> ClimberPositionValues.UP, (value) -> ClimberPositionValues.UP = value);
                builder.addDoubleProperty("Positions/DOWN", () -> ClimberPositionValues.DOWN, (value) -> ClimberPositionValues.DOWN = value);
            }
        }
    }


    public enum ClimberPosition {
        DOWN, UP
    }

    /**
     * Allow for the position values to be changed on the dashboard
     */
    public static class ClimberPositionValues {
        static double UP = -204;
        static double DOWN = -12.24;
        static double RESET = 0;
    }
}
