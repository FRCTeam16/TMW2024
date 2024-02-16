package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.Pivot;

/**
 * The Shooter class represents the shooter subsystem of the robot.
 * It controls the upper and lower shooter motors, the feeder motor,
 * and the aim position of the shooter.
 */
public class Shooter extends SubsystemBase implements Lifecycle, Sendable {
    private final ShooterHelper upper = new ShooterHelper("ShooterSubsystem", "Upper",  new TalonFX(40));
    private final ShooterHelper lower =  new ShooterHelper("ShooterSubsystem", "Lower", new TalonFX(41));
    private final FeederHelper feeder = new FeederHelper("ShooterSubsystem", "Feeder", new TalonFX(42));


    public Shooter(){
        upper.setInvert(true);
    }

    public void runFeeder(){
        feeder.setEnabled(true);
    }

    public void stopFeeder(){
        feeder.setEnabled(false);
    }

    public void runShooterOpenLoop(){
        upper.setOpenLoop(true);
        lower.setOpenLoop(true);
        upper.setEnabled(true);
        lower.setEnabled(true);
    }

    public void runShooter() {
        DataLogManager.log("[Shooter] runShooter");
        upper.setOpenLoop(false);
        lower.setOpenLoop(false);
        lower.setVelocitySetpoint(50);
        upper.setVelocitySetpoint(50);
        upper.setEnabled(true);
        lower.setEnabled(true);
    }
    public void stopShooter(){
        DataLogManager.log("[Shooter] stopShooter");
        upper.setOpenLoop(false);
        lower.setOpenLoop(false);
        lower.setVelocitySetpoint(0);
        upper.setVelocitySetpoint(0);
        upper.setEnabled(false);
        lower.setEnabled(false);
    }

    @Override
    public void periodic(){
        upper.periodic();
        lower.periodic();
        feeder.periodic();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("ShooterSubsystem");
        builder.setActuator(true);
        upper.initSendable(builder);
        lower.initSendable(builder);
        feeder.initSendable(builder);
    }
}
