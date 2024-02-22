package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems;
import frc.robot.subsystems.util.PIDHelper;

import java.util.Optional;

public class Pivot extends SubsystemBase implements Lifecycle, Sendable {

    public static final String SUBSYSTEM_NAME = "Pivot";
    private static final double DEFAULT_OPENLOOP_SPEED = 0.05;
    private final TalonFX motor = new TalonFX(33);
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(1);
    private final DutyCycleOut openLoopOut = new DutyCycleOut(0);
    private final PositionVoltage positionVoltageOut = new PositionVoltage(0);
    private final PIDHelper pidHelper = new PIDHelper(SUBSYSTEM_NAME + "Subsystem/PID");
    private final PIDController pid;
    private final VisionAimManager visionAimManager;
    private final DoubleLogEntry setpointLog;
    private boolean softLimitsEnabled = true;
    private boolean openLoop = true;
    private double speed = 0.0;
    private TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);
    private PivotPosition currentPosition = PivotPosition.Up;


    public Pivot() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        motor.setNeutralMode(NeutralModeValue.Coast);

        pidHelper.initialize(0.02, 0, 0, 0, 0, 0);
        pid = new PIDController(0, 0, 0);
        pidHelper.updatePIDController(pid);

        encoder.setPositionOffset(EncoderConstants.ZERO_OFFSET);

        this.visionAimManager = new VisionAimManager(Subsystems.visionSubsystem.getDefaultLimelight());

        setpointLog = new DoubleLogEntry(DataLogManager.getLog(), "PivotSetpoint");

        this.holdPosition();
        speed = 0.0;
        SmartDashboard.setDefaultNumber(SUBSYSTEM_NAME + "/OpenLoopSpeedx", DEFAULT_OPENLOOP_SPEED);
    }

    @Override
    public void teleopInit() {
        // Set our current position to be our current angle
        this.holdPosition();
        this.goal = new TrapezoidProfile.State(this.getPivotAngleDegrees(), 0);
    }

    public boolean isOpenLoop() {
        return openLoop;
    }

    public void openLoopStop() {
        openLoop = true;
        speed = 0.0;
    }

    public void openLoopUp() {
        DataLogManager.log("[" + SUBSYSTEM_NAME + "] openLoopUp");
        openLoop = true;
        speed = SmartDashboard.getNumber(SUBSYSTEM_NAME + "/OpenLoopSpeedx", DEFAULT_OPENLOOP_SPEED);
    }

    public void openLoopDown() {
        DataLogManager.log("[" + SUBSYSTEM_NAME + "] openLoopDown");
        openLoop = true;
        speed = -SmartDashboard.getNumber(SUBSYSTEM_NAME + "/OpenLoopSpeedx", DEFAULT_OPENLOOP_SPEED);
    }

    public void holdPosition() {
        this.setPivotSetpoint(this.getPivotAngleDegrees());
    }

    public void setPivotPosition(PivotPosition position) {
        if (PivotPosition.VisionAim == position) {
            this.holdPosition();
        } else {
            this.setPivotSetpoint(position.setpoint);
        }
        this.currentPosition = position;
    }

    private double getPivotSetpoint() {
        return this.goal.position;
    }

    private void setPivotSetpoint(double setpoint) {
        DataLogManager.log("[" + SUBSYSTEM_NAME + "] Setting pivot setpoint to: " + setpoint);
        if (setpoint >= 80.0001 || setpoint <= -0.0001) {
            DataLogManager.log("[" + SUBSYSTEM_NAME + "] INVALID SETPOINT REQUESTED, IGNORING");
            return;
        }
        openLoop = false;
        setpointLog.append(setpoint);
        this.goal = new TrapezoidProfile.State(setpoint, 0);
    }

    private boolean isVisionAiming() {
        return (PivotPosition.VisionAim == this.currentPosition);
    }

    double getSpeed() {
        return speed;
    }

    void setSpeed(double speed) {
        this.speed = speed;
    }

    /**
     * Returns the angle of the pivoted arm, with 0 degrees at the top
     */
    public double getPivotAngleDegrees() {
        return -encoder.get() * 360.0;
    }

    public double getPivotEncoderPosition() {
        return encoder.get();
    }

    public double getPivotEncoderAbsolutePosition() {
        return encoder.getAbsolutePosition();
    }

    public double getPivotEncoderPositionOffset() {
        return encoder.getPositionOffset();
    }

    @Override
    public void periodic() {

        if (this.getPivotAngleDegrees() > EncoderConstants.MAX_ANGLE || this.getPivotAngleDegrees() < EncoderConstants.MIN_ANGLE) {
            DataLogManager.log("[" + SUBSYSTEM_NAME + "] SOFT LIMIT BREAK, setting output to 0");
            motor.setControl(openLoopOut.withOutput(0));
            return;
        }

        if (openLoop) {
            motor.setControl(openLoopOut.withOutput(speed));
        } else {
            if (pidHelper.updateValuesFromDashboard()) {
                pidHelper.updatePIDController(pid);
            }
            final double output;

            if (isVisionAiming()) {
                Optional<VisionAimManager.VisionAimResult> result = visionAimManager.calculate();
            }
            {
                output = pid.calculate(this.getPivotAngleDegrees(), goal.position);
            }
            motor.setControl(openLoopOut.withOutput(output));
        }
    }

    public Double getMotorPosition() {
        return this.motor.getPosition().getValue();
    }

    public Double getMotorVelocity() {
        return this.motor.getVelocity().getValue();
    }

    public Double getMotorVoltage() {
        return this.motor.getMotorVoltage().getValue();
    }

    public Double getMotorSupplyCurrent() {
        return this.motor.getStatorCurrent().getValue();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(SUBSYSTEM_NAME);

        builder.addBooleanProperty("OpenLoop", this::isOpenLoop, null);
        builder.addDoubleProperty("Goal", this::getPivotSetpoint, this::setPivotSetpoint);
        builder.addDoubleProperty("Speed", this::getSpeed, null /*this::setSpeed*/);

        builder.addDoubleProperty("Motor/Position", this::getMotorPosition, null);
        builder.addDoubleProperty("Motor/Velocity", this::getMotorVelocity, null);
        builder.addDoubleProperty("Motor/Voltage", this::getMotorVoltage, null);
        builder.addDoubleProperty("Motor/SupplyCurrent", this::getMotorSupplyCurrent, null);

        builder.addDoubleProperty("Encoder/Pos", this::getPivotEncoderPosition, null);
        builder.addDoubleProperty("Encoder/Angle", this::getPivotAngleDegrees, null);
        builder.addDoubleProperty("Encoder/AbsolutePos", this::getPivotEncoderAbsolutePosition, null);
        builder.addDoubleProperty("Encoder/Offset", this::getPivotEncoderPositionOffset, null);
    }

    public Command moveToPositionCmd(PivotPosition position) {
        return Commands.runOnce(() -> this.setPivotPosition(position));
    }


    public enum PivotPosition {
        StartingPosition(0),
        Horizontal(0),
        FeedPosition(21.0),
        ScoringAngle(30),
        Up(70),
        VisionAim(0);

        public final double setpoint;

        PivotPosition(double setpoint) {
            this.setpoint = setpoint;
        }
    }

    private static class EncoderConstants {
        static final double ZERO_OFFSET = 0.3285406832135171;
        static final double MIN_ANGLE = -1.0;
        static final double MAX_ANGLE = 81.75;      // 0.12004950300123758
    }

}