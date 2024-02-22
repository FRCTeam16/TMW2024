package frc.robot.subsystems.trap;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Lifecycle;

public class Trap extends SubsystemBase implements Lifecycle, Sendable {
    public static final String SUBSYSTEM_NAME = "TrapSubsystem";
    private TrapExtender extender = new TrapExtender(new TalonFX(21));
    private TrapPivot pivot = new TrapPivot(new TalonFX(22));
    private Servo finger1 = new Servo(1); // I dont know what 1 means but yea
    private Servo finger2 = new Servo(2);
    private Servo wristRotate = new Servo(3);

    @Override
    public void teleopInit() {
        this.getExtender().teleopInit();
        this.getPivot().teleopInit();
    }

    @Override
    public void autoInit() {
        this.getExtender().autoInit();
        this.getPivot().autoInit();
    }

    public TrapExtender getExtender() {
        return extender;
    }

    public TrapPivot getPivot() {
        return pivot;
    }

    @Override
    public void periodic() {
        extender.periodic();
        pivot.periodic();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        if (Constants.UseSendables) {
            builder.setSmartDashboardType(SUBSYSTEM_NAME);
            // TODO: expose finger values
            pivot.initSendable(builder);
            extender.initSendable(builder);
        }
    }
}
