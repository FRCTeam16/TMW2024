package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.VisionAimManager;

/**
 * The Shooter class represents the shooter subsystem of the robot.
 * It controls the upper and lower shooter motors, the feeder motor,
 * and the aim position of the shooter.
 */
public class Shooter extends SubsystemBase implements Lifecycle, Sendable {
    public static final int DEFAULT_VELOCITY_SETPOINT = 25;
    private final DigitalInput noteFeedStop = new DigitalInput(2);
    private final ShooterHelper upper = new ShooterHelper("ShooterSubsystem", "Upper",  new TalonFX(40));
    private final ShooterHelper lower =  new ShooterHelper("ShooterSubsystem", "Lower", new TalonFX(41));
    private final FeederHelper feeder = new FeederHelper("ShooterSubsystem", "Feeder", new TalonFX(42), noteFeedStop);


    public Shooter(){
        upper.setInvert(true);
    }


    public void applyShootingProfile(VisionAimManager.ShootingProfile shootingProfile) {
        upper.setVelocitySetpoint(shootingProfile.upperSpeed());
        lower.setVelocitySetpoint(shootingProfile.lowerSpeed());
    }

    @Override
    public void teleopInit() {
        upper.setOpenLoop(true);
        lower.setOpenLoop(true);
        upper.setEnabled(false);
        lower.setEnabled(false);
        feeder.setEnabled(false);
    }

    @Override
    public void autoInit() {
        upper.setOpenLoop(true);
        lower.setOpenLoop(true);
        upper.setEnabled(false);
        lower.setEnabled(false);
        feeder.setEnabled(false);
    }

    /**
     * FIXME DEBUG
     */
    public FeederHelper getFeeder() {
        return this.feeder;
    }

    public boolean isNoteDetected() {
        return getFeeder().isNoteDetected();
    }


    /**
     *  testing
     */
    public void runFeeder(){
        feeder.setOpenLoopSetpoint(-0.50);
//        feeder.setEnabled(true);
    }

    public void stopFeeder(){
        feeder.setOpenLoopSetpoint(0.0);
//        feeder.setEnabled(false);
    }

    /**
     * Runs the feeder until a note is detected
     */
    public void feedNote() {
        feeder.receiveFromIntake();
    }

    public void shoot() {
        feeder.shoot();
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
        lower.setVelocitySetpoint(DEFAULT_VELOCITY_SETPOINT);
        upper.setVelocitySetpoint(DEFAULT_VELOCITY_SETPOINT);
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
        if (Constants.UseSendables) {
            builder.setSmartDashboardType("ShooterSubsystem");
            builder.setActuator(true);
            upper.initSendable(builder);
            lower.initSendable(builder);
            feeder.initSendable(builder);
        }
    }
}
