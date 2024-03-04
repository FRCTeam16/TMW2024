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
import edu.wpi.first.wpilibj2.command.Commands;
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

    private final SoftLimitValues softLimits = new SoftLimitValues(0, -320);

    //
    // Open Loop Handling
    //
    private boolean openLoop = false;
    private double openLoopSpeed = 0.0;
    private ClimberPosition currentPosition = ClimberPosition.DOWN;
    // Closed loop setpoint
    private double setpoint = 0.0;
    private boolean softLimitsEnabled = true;

    public Climber() {
//    climberDrive.setNeutralMode(NeutralModeValue.Brake);
        pidHelper.initialize(2.0, 0, 0, 0, 0, 0);
        config = new TalonFXConfiguration()
                .withSoftwareLimitSwitch(
                        new SoftwareLimitSwitchConfigs()
                                .withForwardSoftLimitThreshold(softLimits.forwardSoftLimitThreshold()).withForwardSoftLimitEnable(true)
                                .withReverseSoftLimitThreshold(softLimits.reverseSoftLimitThreshold()).withReverseSoftLimitEnable(true))
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
        openLoop = true;
        openLoopSpeed = climberOpenLoopSpeeds.getUpSpeed();
    }

    public void openLoopDown() {
        openLoop = true;
        openLoopSpeed = climberOpenLoopSpeeds.getDownSpeed();
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

    private void setSoftLimits(boolean enable) {
        climberDrive.getConfigurator().apply(
                new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitThreshold(softLimits.forwardSoftLimitThreshold()).withForwardSoftLimitEnable(enable)
                        .withReverseSoftLimitThreshold(softLimits.reverseSoftLimitThreshold()).withReverseSoftLimitEnable(enable));
    }


    public void stopOpenLoop() {
        if (!softLimitsEnabled) {
            softLimitsEnabled = true;
            setSoftLimits(true);
        }
        openLoop = true;
        openLoopSpeed = 0.0;
    }

    public Command moveToStateCmd(ClimberPosition climberPosition) {
        return Commands.runOnce(() -> setClimberPosition(climberPosition));
    }

    public ClimberPosition getClimberPosition() {
        return this.currentPosition;
    }

    public void setClimberPosition(ClimberPosition position) {
        this.currentPosition = position;
        this.setClimberSetpoint(position.setpoint);
    }

    private void setClimberSetpoint(double setpoint) {
        openLoop = false;
        this.setpoint = setpoint;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    @Override
    public void periodic() {
        if (pidHelper.updateValuesFromDashboard()) {
            pidHelper.updateConfiguration(config.Slot0);
            climberDrive.getConfigurator().apply(config.Slot0);
        }

        if (openLoop) {
            climberDrive.setControl(openLoopOut.withOutput(openLoopSpeed)
                    .withLimitForwardMotion(softLimitsEnabled)
                    .withLimitReverseMotion(softLimitsEnabled));
        } else {
            climberDrive.setControl(positionOut.withPosition(this.setpoint));
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        if (Constants.Dashboard.UseSendables) {
            builder.setSmartDashboardType(SUBSYSTEM_NAME);
            builder.addStringProperty("ClimberPosition", () -> this.getClimberPosition().name(), null);
            builder.addDoubleProperty("Setpoint", this::getSetpoint, this::setSetpoint);
            builder.addDoubleProperty("Encoder", () -> this.climberDrive.getPosition().getValue(), null);
            builder.addDoubleProperty("Velocity", () -> this.climberDrive.getVelocity().getValue(), null);
            builder.addDoubleProperty("Acceleration", () -> this.climberDrive.getAcceleration().getValue(), null);
            builder.addDoubleProperty("OpenLoop/Speed", () -> this.openLoopSpeed, null);

            if (Constants.Dashboard.ConfigurationMode) {
                builder.addBooleanProperty("Pivot/OpenLoopEnabled", this::isOpenLoop, this::setOpenLoop);
                builder.addDoubleProperty("Pivot/UpSpeed", this.climberOpenLoopSpeeds::getUpSpeed, this.climberOpenLoopSpeeds::setUpSpeed);
                builder.addDoubleProperty("Pivot/DownSpeed", this.climberOpenLoopSpeeds::getDownSpeed, this.climberOpenLoopSpeeds::setDownSpeed);
            }
        }
    }




    public enum ClimberPosition {
        DOWN(0),
        UP(-315);

        public final double setpoint;

        private ClimberPosition(double setpoint) {
            this.setpoint = setpoint;
        }
    }
}