package frc.robot;

import java.util.ArrayList;
import java.util.List;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.RotationController;
import frc.robot.subsystems.DMS.LEDSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.Intake;

/**
 * Global subsystem declaration and tracking to allow easier injection
 * for commands
 */
public class Subsystems {
    private static Subsystems instance;
    public static CommandSwerveDrivetrain swerveSubsystem;
    public static VisionSubsystem visionSubsystem;
    public static LEDSubsystem ledSubsystem;
    public static Intake intake;

    // Utility
    public static RotationController rotationController = new RotationController();

    public static List<Lifecycle> lifecycleSubsystems = new ArrayList<>();

    private Subsystems() {
        swerveSubsystem = TunerConstants.DriveTrain;
        visionSubsystem = new VisionSubsystem();
        ledSubsystem = new LEDSubsystem();
        //TODO add if to prevent comp bot subsystem calls on practice bot
        intake = new Intake();
        
        lifecycleSubsystems.add(visionSubsystem);
        lifecycleSubsystems.add(ledSubsystem);
        lifecycleSubsystems.add(intake);
    }


    public static Subsystems getInstance() {
        if (instance == null) {
            instance = new Subsystems();
        }
        return instance;
    }
}
