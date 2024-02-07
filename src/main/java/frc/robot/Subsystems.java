package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DMS.LEDSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.RotationController;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionTypes;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;


/**
 * The Subsystems class represents a collection of subsystems used in the robot.
 * It provides singleton access to all the subsystem instances and manages their lifecycle.
 */
@SuppressWarnings("InstantiationOfUtilityClass")
public class Subsystems {
    public static CommandSwerveDrivetrain swerveSubsystem;
    public static VisionSubsystem visionSubsystem;
    public static LEDSubsystem ledSubsystem;
    public static Intake intake;
    public static Shooter shooter;
    // Utility
    public static RotationController rotationController = new RotationController();
    public static List<Lifecycle> lifecycleSubsystems = new ArrayList<>();
    private static Subsystems instance;

    private Subsystems() {
        swerveSubsystem = TunerConstants.DriveTrain;
        visionSubsystem = new VisionSubsystem(
                Stream.of(
                        new VisionTypes.LimelightInfo("limelight", 10, 0),
                        new VisionTypes.LimelightInfo("note", 10, 0))
                        .map(Limelight::new).collect(Collectors.toSet()));
        ledSubsystem = new LEDSubsystem();
        //TODO add if to prevent comp bot subsystem calls on practice bot
        intake = new Intake();
        shooter = new Shooter();

        lifecycleSubsystems.add(visionSubsystem);
        lifecycleSubsystems.add(ledSubsystem);
        lifecycleSubsystems.add(intake);
        lifecycleSubsystems.add(shooter);

        SmartDashboard.putData("ShooterSubsystem", shooter);
    }


    public static Subsystems getInstance() {
        if (instance == null) {
            instance = new Subsystems();
        }
        return instance;
    }
}
