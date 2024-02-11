package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.util.PIDHelper;

public class Pivot extends SubsystemBase implements Lifecycle, Sendable {

    private final TalonFX motor = new TalonFX(33);
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(1);

    private boolean softLimitsEnabled = true;

    private static final double DEFAULT_OPENLOOP_SPEED = 0.05;

    private boolean openLoop = true;
    private double speed = 0.0;

    private final DutyCycleOut openLoopOut = new DutyCycleOut(0);
    private final PositionVoltage positionVoltageOut = new PositionVoltage(0);

    private final PIDHelper pidHelper = new PIDHelper("Pivot");

    private PIDController pid;
    private TrapezoidProfile.State goal = new TrapezoidProfile.State(0,0);


    public enum PivotPosition {
        StartingPosition(0),
        Horizontal(0),
        FeedPosition(20.0),
        ScoringAngle(30),
        Up(70);

        public final double setpoint;

        private PivotPosition(double setpoint) {
            this.setpoint = setpoint;
        }
    }


    private static class EncoderCounts {
        static final double ZERO = 0.3285406832135171;
        static final double MIN_ANGLE = -1.0;
        static final double MAX_ANGLE = 81.75;      // 0.12004950300123758
    }

    public Pivot() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        motor.setNeutralMode(NeutralModeValue.Coast);

        pidHelper.initialize(0.01, 0, 0, 0, 0, 0);
        pid = new PIDController(0,0,0);
        pidHelper.updatePIDController(pid);

        encoder.setPositionOffset(EncoderCounts.ZERO);

        SmartDashboard.setDefaultNumber("Pivot/OpenLoopSpeedx", DEFAULT_OPENLOOP_SPEED);

        this.holdPosition();

        openLoop = true;
        speed = 0.0;
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
        DataLogManager.log("[Pivot] openLoopUp");
        openLoop = true;
        speed = SmartDashboard.getNumber("Pivot/OpenLoopSpeedx", DEFAULT_OPENLOOP_SPEED);
    }

    public void openLoopDown() {
        DataLogManager.log("[Pivot] openLoopDown");
        openLoop = true;
        speed = -SmartDashboard.getNumber("Pivot/OpenLoopSpeedx", DEFAULT_OPENLOOP_SPEED);
    }

    public void holdPosition() {
        this.setPivotSetpoint(this.getPivotAngleDegrees());
    }

    public void setPivotPosition(PivotPosition position) {
        this.setPivotSetpoint(position.setpoint);
    }

    private double getPivotSetpoint() {
        return this.goal.position;
    }

    private void setPivotSetpoint(double setpoint) {
        DataLogManager.log("[Pivot] Setting pivot setpoint to: " + setpoint);
        if (setpoint >= 80.0001 || setpoint <= -0.0001) {
            DataLogManager.log("[Pivot] INVALID SETPOINT REQUESTED, IGNORING");
            return;
        }
        openLoop = false;
        this.goal = new TrapezoidProfile.State(setpoint, 0);
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
//        if (this.goal.position > EncoderCounts.MAX_ANGLE || this.goal.position < EncoderCounts.MIN_ANGLE) {
//
//        }

        if (this.getPivotAngleDegrees() > EncoderCounts.MAX_ANGLE || this.getPivotAngleDegrees() < EncoderCounts.MIN_ANGLE) {
            DataLogManager.log("[Pivot] SOFT LIMIT BREAK");
            motor.setControl(openLoopOut.withOutput(0));
            return;
        }


        if (openLoop) {
            motor.setControl(openLoopOut.withOutput(speed));
        } else {
            pidHelper.updateValuesFromDashboard();
            pidHelper.updatePIDController(pid);
            double output = pid.calculate(this.getPivotAngleDegrees(), goal.position);
            motor.setControl(openLoopOut.withOutput(output));
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Pivot");

        builder.addBooleanProperty("OpenLoop", this::isOpenLoop, null);
        builder.addDoubleProperty("Goal", this::getPivotSetpoint, this::setPivotSetpoint);
        builder.addDoubleProperty("Speed", this::getSpeed, null /*this::setSpeed*/);

//        builder.addDoubleProperty("Motor/Position", () -> this.motor.getPosition().getValue(), null);
//        builder.addDoubleProperty("Motor/Velocity", () -> this.motor.getVelocity().getValue(), null);
//        builder.addDoubleProperty("Motor/Voltage", () -> this.motor.getMotorVoltage().getValue(), null);
//        builder.addDoubleProperty("Motor/Amps", () -> this.motor.getStatorCurrent().getValue(), null);

        builder.addDoubleProperty("Encoder/Pos", this::getPivotEncoderPosition, null);
        builder.addDoubleProperty("Encoder/Angle", this::getPivotAngleDegrees, null);
        builder.addDoubleProperty("Encoder/AbsolutePos", this::getPivotEncoderAbsolutePosition, null);
        builder.addDoubleProperty("Encoder/Offset", this::getPivotEncoderPositionOffset, null);
    }

}