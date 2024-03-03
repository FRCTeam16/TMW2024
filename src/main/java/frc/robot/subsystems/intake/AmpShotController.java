package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

/**
 * Tr
 */
public class AmpShotController implements Sendable {
    private final DutyCycleEncoder encoder;

    private PIDController pid = new PIDController(96, 0, 8);
    private TrapezoidProfile.State goal;

    private double clamp = 2;

    public AmpShotController(DutyCycleEncoder encoder, double setpoint) {
        this.encoder = encoder;
        this.goal = new TrapezoidProfile.State(setpoint, 0);
    }

    public double calculate() {
        double rawValue = -pid.calculate(encoder.get(), goal.position);
        return MathUtil.clamp(rawValue, -clamp, clamp);
    }

    @Override
    public void initSendable(SendableBuilder sendableBuilder) {

        sendableBuilder.addDoubleProperty("Pivot/ASC/Setpoint", () -> goal.position, (value) -> goal = new TrapezoidProfile.State(value, 0));
        sendableBuilder.addDoubleProperty("Pivot/ASC/ProcessVariable", encoder::get, null);

        sendableBuilder.addDoubleProperty("Pivot/ASC/kP", () -> pid.getP(), (value) -> pid.setP(value));
        sendableBuilder.addDoubleProperty("Pivot/ASC/kI", () -> pid.getI(), (value) -> pid.setI(value));
        sendableBuilder.addDoubleProperty("Pivot/ASC/kD", () -> pid.getD(), (value) -> pid.setD(value));

        sendableBuilder.addDoubleProperty("Pivot/ASC/Clamp", () -> clamp, (value) -> clamp = value);
    }
}
