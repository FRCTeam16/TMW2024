package frc.robot.auto;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.auto.strategies.*;
import frc.robot.commands.auto.*;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.pose.PoseManager;

import java.util.HashMap;
import java.util.function.Supplier;

/**
 * Manages the autonomous mode of the robot.
 * <p>
 * Steps to add a new strategy:
 * 1. Create a new command that implements the strategy, most likely extending
 * AutoPathStrategy or SequentialCommandGroup
 * 2. Add a new enum value to AutoStrategies
 * 3. Register the strategy in the registerStrategies method
 * 4. Register the path in the registerAutoPaths method
 * 5. Register any named commands in the registerNamedCommands method
 */
public class AutoManager {

    private static final PathRegistry pathRegistry = new PathRegistry();
    private final SendableChooser<AutoStrategies> chooser = new SendableChooser<>();
    private final HashMap<AutoStrategies, Command> strategyLookup = new HashMap<>();

    public AutoManager() {
    }

    /**
     * Initializes the autonomous manager.
     * Must be called before using the manager.
     * Put in place to work around some static reference ordering issues.
     */
    public void initialize() {
        registerNamedCommands();
        registerAutoPaths();
        registerStrategies();

        // Send selector Dashboard. If it doesn't show in SD, you may need to change the
        // name here.
        SmartDashboard.putData("Auto Selector", chooser);
    }

    /**
     * Returns the command for the given path name
     *
     * @param pathName the name of the path
     * @return the command for the path
     */
    public Command getAutoPath(String pathName) {
        return pathRegistry.getPath(pathName);
    }

    /**
     * Registers the autonomous strategies with the chooser and the lookup map
     */
    private void registerStrategies() {
        // registerStrategy("Debug Auto", AutoStrategies.DebugAuto, DebugAuto::new);
        // registerStrategy("Test Path", AutoStrategies.DebugTestPath, () -> new DebugPathAuto("TestStageLeft"));
        // registerStrategy("Test MultiPath", AutoStrategies.DebugTestMultipath,
                // () -> new DebugPathAuto("Multipoint Test"));
        // registerStrategy("Test Curly", AutoStrategies.DebugCurly, () -> new DebugPathAuto("TestCurly"));
        // registerStrategy("Test Wavy", AutoStrategies.DebugWavy, () -> new DebugPathAuto("TestWave"));
        // registerStrategy("Tests2", AutoStrategies.DebugTests2, () -> new DebugPathAuto("Test2"));
        // registerStrategy("DCL-BCL", AutoStrategies.DCL_BCL, () -> new DebugPathAuto("DCL_BCL"));

        // registerStrategy("Under The Bridge", AutoStrategies.UnderTheBridge, UnderTheBridge::new);
        registerStrategy("Tab", AutoStrategies.Tab, () -> new Tab(Tab.TabVersion.OffsetStart));
        registerStrategy("TabStraight", AutoStrategies.Tab, () -> new Tab(Tab.TabVersion.StraightStart));
        registerStrategy("DropShot", AutoStrategies.DropShot, DropShot::new);
        // registerStrategy("CrossWing", AutoStrategies.CrossWing, CrossWingShot::new);
    }

    /**
     * Registers Named commands for use in PathPlanner GUI
     */
    private void registerNamedCommands() {
        NamedCommands.registerCommand("rotate45", new RotateToAngle(-45));
        NamedCommands.registerCommand("StartIntake", Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Pickup));
        NamedCommands.registerCommand("StartShooter", new EnableShooterCommand());
        NamedCommands.registerCommand("StopShooter", new EnableShooterCommand(false));
        NamedCommands.registerCommand("StartAutoAim",
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.ShooterAimVision));
        NamedCommands.registerCommand("StartDrive", Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Drive));
        NamedCommands.registerCommand("FeedNote", new FeedNoteInAuto());
        NamedCommands.registerCommand("ShootNote",
                Commands.runOnce(Subsystems.shooter::shoot).andThen(new WaitCommand(0.2)));

        NamedCommands.registerCommand("WaitIntakeInPosition",
                new WaitIntakeInPosition(IntakePivot.IntakePosition.Zero).withTimeout(1.0));

        NamedCommands.registerCommand("WaitForNote", new WaitShooterHasNote().withTimeout(1.0));

        NamedCommands.registerCommand("WaitNoteThenAutoAim",
                Commands.waitUntil(Subsystems.shooter::isNoteDetected).withTimeout(1.0)
                        .andThen(Subsystems.poseManager.getPoseCommand(PoseManager.Pose.ShooterAimVision)));

        // UTB
        NamedCommands.registerCommand("UTBSetThirdShotPivot",
                Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.Custom));
        NamedCommands.registerCommand("UTBSetSecondShotSpeed",
                Commands.runOnce(() -> Subsystems.shooter.applyShootingProfile(UnderTheBridge.SecondShotProfile)));
        NamedCommands.registerCommand("UTBSetThirdShotSpeed",
                Commands.runOnce(() -> Subsystems.shooter.applyShootingProfile(UnderTheBridge.ThirdShotProfile)));

        // DropShot
        NamedCommands.registerCommand("BloopShot", new BloopShot());
        
        }

    /**
     * Registers the autonomous paths with the path registry
     */
    private void registerAutoPaths() {
        UnderTheBridge.registerAutoPaths(pathRegistry);
        Tab.registerAutoPaths(pathRegistry);
        DropShot.registerAutoPaths(pathRegistry);
    }

    // Register the strategy with the chooser and the lookup map
    private void registerStrategy(String displayName, AutoStrategies strategyEnum, Supplier<Command> strategy) {
        registerStrategy(displayName, strategyEnum, strategy, false);
    }

    // Register the strategy with the chooser and the lookup map
    private void registerStrategy(String displayName, AutoStrategies strategyEnum, Supplier<Command> strategy,
                                  boolean isDefault) {
        if (isDefault) {
            chooser.setDefaultOption(displayName, strategyEnum);
        } else {
            chooser.addOption(displayName, strategyEnum);
        }
        strategyLookup.put(strategyEnum, strategy.get());
    }

    /**
     * Returns the currently selected autonomous strategy
     *
     * @return the currently selected autonomous strategy
     */
    public Command getSelectedAutoStrategy() {
        Command selected = null;

        if (strategyLookup.containsKey(chooser.getSelected())) {
            selected = strategyLookup.get(chooser.getSelected());
        } else {
            // Missing autonomous key in lookup map
            selected = Commands.print("[AutoManager] ERROR: Could not find requested auto: " + chooser.getSelected());
        }
        return selected;
    }

    public void showSelectedAuto() {
        var selected = chooser.getSelected();
        SmartDashboard.putString("Selected Auto", (selected != null) ? selected.name() : "Unknown");
    }

    public enum AutoStrategies {
        DebugAuto,
        DebugTestPath,
        DebugTestMultipath,
        DebugCurly,
        DebugWavy,
        DebugTests2,
        DCL_BCL,
        UnderTheBridge,
        DropShot, 
        Tab,
        CrossWing
    }

}
