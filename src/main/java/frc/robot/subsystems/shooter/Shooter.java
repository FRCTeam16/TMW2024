package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.VisionAimManager;
import frc.robot.subsystems.util.BSLogger;

/**
 * The Shooter class represents the shooter subsystem of the robot.
 * It controls the upper and lower shooter motors, the feeder motor,
 * and the aim position of the shooter.
 */
public class Shooter extends SubsystemBase implements Lifecycle, Sendable {
    public static final int DEFAULT_VELOCITY_SETPOINT = 25;
    private final DigitalInput noteFeedStop = new DigitalInput(2);
    private final DigitalInput slowFeedSensor = new DigitalInput(3);
    private final ShooterHelper upper = new ShooterHelper("ShooterSubsystem", "Upper",  new TalonFX(40));
    private final ShooterHelper lower =  new ShooterHelper("ShooterSubsystem", "Lower", new TalonFX(41));
    private final FeederHelper feeder = new FeederHelper("ShooterSubsystem", "Feeder", new TalonFX(42), noteFeedStop, slowFeedSensor);
    public final VisionAimManager.ShootingProfile BloopProfile =
            new VisionAimManager.ShootingProfile(Pivot.PivotPosition.Horizontal.setpoint, 10, 10);

    public static final VisionAimManager.ShootingProfile BigShotProfile =
            new VisionAimManager.ShootingProfile(35, 25, 25);

    public static final VisionAimManager.ShootingProfile SubShotProfile =
            new VisionAimManager.ShootingProfile(50, 25, 25);

    public static final VisionAimManager.ShootingProfile ShootOverSmileyProfile =
            new VisionAimManager.ShootingProfile(15.75, 65, 61);


    public Shooter(){
        upper.setInvert(true);
    }


    public void applyShootingProfile(VisionAimManager.ShootingProfile shootingProfile) {
        upper.setVelocitySetpoint(shootingProfile.upperSpeed());
        lower.setVelocitySetpoint(shootingProfile.lowerSpeed());
    }

    @Override
    public void teleopInit() {
        //runShooter(); //switching initStart to first time intake is lowered
    }

    @Override
    public void autoInit() {
        upper.setOpenLoop(true);
        lower.setOpenLoop(true);
        upper.setEnabled(false);
        lower.setEnabled(false);
        feeder.setEnabled(false);
    }

    public FeederHelper getFeeder() {
        return this.feeder;
    }

    public boolean isNoteDetected() {
        return getFeeder().isNoteDetected();
    }

    /**
     * Checks the Shooter upper and lower are at speed
     * @return
     */
    public boolean isAtSpeed() {
        return upper.isAtSpeed() && lower.isAtSpeed();
    }

    public boolean isFeederAtSpeed() {
        return feeder.isAtSpeed();
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
    @Deprecated public void feedNote() {
        feeder.setSlowFeedMode(false);
        feeder.receiveFromIntake();
    }

    public static boolean isReadyToShoot(){
         // Has Target
        return  RobotContainer.isUseVisionAlignment() &&
                Subsystems.visionSubsystem.hasTarget() &&
                Subsystems.pivot.isInPosition() &&
                Subsystems.shooter.isAtSpeed();
    }

    void shoot() {
        feeder.shoot();
    }

    public Command shootCmd() {
        Command cmd = Commands.sequence(
                Commands.runOnce(feeder::enableCoastMode),
                Commands.runOnce(this::shoot),
                new WaitCommand(0.2),
                Commands.runOnce(feeder::enableBreakMode)
        );
        cmd.addRequirements(Subsystems.shooter);
        return cmd;

    }


    public void runShooterOpenLoop(){
        upper.setOpenLoop(true);
        lower.setOpenLoop(true);
        upper.setEnabled(true);
        lower.setEnabled(true);
    }

    public void runShooter() {
        BSLogger.log("Shooter", "runShooter");
        feeder.setEnabled(true);
        upper.setOpenLoop(false);
        lower.setOpenLoop(false);
        lower.setVelocitySetpoint(DEFAULT_VELOCITY_SETPOINT);
        upper.setVelocitySetpoint(DEFAULT_VELOCITY_SETPOINT);
        upper.setEnabled(true);
        lower.setEnabled(true);
    }
    public void stopShooter() {
        BSLogger.log("Shooter", "stopShooter");
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
        if (Constants.Dashboard.UseSendables) {
            builder.setSmartDashboardType("ShooterSubsystem");
            builder.setActuator(true);
            upper.initSendable(builder);
            lower.initSendable(builder);
            feeder.initSendable(builder);
            builder.addBooleanProperty("isReadyToShoot", Shooter::isReadyToShoot, null); // good
            builder.addBooleanProperty("isAtSpeed", this::isAtSpeed, null);
            builder.addBooleanProperty("HasTarget", Subsystems.visionSubsystem::hasTarget, null);
        }
    }

    public boolean isEnabled() {
        return feeder.isEnabled() && upper.isEnabled() && lower.isEnabled();
    }
}
