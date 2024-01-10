package frc.robot;

import java.util.ArrayList;
import java.util.List;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.DMS.LEDSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * Global subsystem declaration and tracking to allow easier injection
 * for commands
 */
public class Subsystems {
    private static Subsystems instance;
    public static CommandSwerveDrivetrain swerveSubsystem;
    public static VisionSubsystem visionSubsystem;
    public static LEDSubsystem ledSubsystem;
    
    
    public static List<Lifecycle> lifecycleSubsystems = new ArrayList<>();

    private Subsystems() {
        swerveSubsystem = TunerConstants.DriveTrain;
        visionSubsystem = new VisionSubsystem();
        ledSubsystem = new LEDSubsystem();

        lifecycleSubsystems.add(visionSubsystem);
        lifecycleSubsystems.add(ledSubsystem);
    }

    public static Subsystems getInstance() {
        if (instance == null) {
            instance = new Subsystems();
        }
        return instance;
    }
}
