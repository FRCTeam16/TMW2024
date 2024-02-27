package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.AutoManager;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.RotationController;
import frc.robot.subsystems.DMS.LEDSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pose.PoseManager;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.trap.Trap;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionTypes;

/**
 * The Subsystems class represents a collection of subsystems used in the robot.
 * It provides singleton access to all the subsystem instances and manages their
 * lifecycle.
 */
@SuppressWarnings("InstantiationOfUtilityClass")
public class Subsystems {
    public static CommandSwerveDrivetrain swerveSubsystem;
    public static VisionSubsystem visionSubsystem;
    public static LEDSubsystem ledSubsystem;
    public static Intake intake;
    public static Shooter shooter;
    public static Pivot pivot;
    public static Climber climber;
    public static Trap trap;

    public static List<Lifecycle> lifecycleSubsystems = new ArrayList<>();

    //
    // Utility classes
    //
    public static PoseManager poseManager;
    public static RotationController rotationController;

    // Warning: This must be created after everything else to ensure all subsystems
    // are registered
    public static AutoManager autoManager;

    private static Subsystems instance;

    private Subsystems() {
        swerveSubsystem = TunerConstants.DriveTrain;
        visionSubsystem = new VisionSubsystem(
                Stream.of(
                        new VisionTypes.LimelightInfo("limelight", 13.1, 25.71))
                        // new VisionTypes.LimelightInfo("notelight", 10, 0))
                        .map(Limelight::new).collect(Collectors.toSet()));
        ledSubsystem = new LEDSubsystem();
        // TODO add if to prevent comp bot subsystem calls on practice bot
        intake = new Intake();
        shooter = new Shooter();
        pivot = new Pivot();
        climber = new Climber();
        // trap = new Trap();

        lifecycleSubsystems.add(visionSubsystem);
        lifecycleSubsystems.add(ledSubsystem);
        lifecycleSubsystems.add(intake);
        lifecycleSubsystems.add(shooter);
        lifecycleSubsystems.add(pivot);
        lifecycleSubsystems.add(climber);
        // lifecycleSubsystems.add(trap);

        SmartDashboard.putData("ShooterSubsystem", shooter);
        SmartDashboard.putData("PivotSubsystem", pivot);
        SmartDashboard.putData("IntakeSubsystem", intake);
        SmartDashboard.putData("ClimberSubsystem", climber);
        // SmartDashboard.putData("TrapSubsystem", trap);

        createUtilitySubsystems();
    }

    private void createUtilitySubsystems() {
        poseManager = new PoseManager();
        rotationController = new RotationController();
        autoManager = new AutoManager();
        autoManager.initialize();
    }

    public static Subsystems getInstance() {
        if (instance == null) {
            instance = new Subsystems();
        }
        return instance;
    }
}
