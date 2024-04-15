package frc.robot.auto;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems;
import frc.robot.auto.strategies.*;
import frc.robot.commands.auto.*;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.VisionAimManager;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.pose.PoseManager;

import javax.naming.Name;
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
        registerStrategy("Tab", AutoStrategies.Tab, Tab::new);
        registerStrategy("DropShot", AutoStrategies.DropShot, DropShot::new);
        // registerStrategy("CrossWing", AutoStrategies.CrossWing, CrossWingShot::new);
        registerStrategy("BlazeBA", AutoStrategies.Blaze, () -> new Blaze(Blaze.BlazeRoute.BlazeBA));
        registerStrategy("BlazeAB", AutoStrategies.Blaze, () -> new Blaze(Blaze.BlazeRoute.BlazeAB));
        registerStrategy("BlazeBC", AutoStrategies.Blaze, () -> new Blaze(Blaze.BlazeRoute.BlazeBC));
        registerStrategy("BlazeCB", AutoStrategies.Blaze, () -> new Blaze(Blaze.BlazeRoute.BlazeCB));
        registerStrategy("BlazeAC", AutoStrategies.Blaze, () -> new Blaze(Blaze.BlazeRoute.BlazeAC));
        registerStrategy("SL1toSL3", AutoStrategies.SL1toSL3, SL1toSL3::new);
    }

    /**
     * Registers Named commands for use in PathPlanner GUI
     */
    private void registerNamedCommands() {
        NamedCommands.registerCommand("rotate45", new RotateToAngle(-45));
        NamedCommands.registerCommand("StartShooter", new EnableShooterCommand());
        NamedCommands.registerCommand("StopShooter", new EnableShooterCommand(false));
        NamedCommands.registerCommand("StartAutoAim",
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.ShooterAimVision));
        NamedCommands.registerCommand("StartDrive", Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Drive));
        NamedCommands.registerCommand("ShootNote", Subsystems.shooter.shootCmd());

        NamedCommands.registerCommand("WaitIntakeInPosition",
                new WaitIntakeInPosition(IntakePivot.IntakePosition.Zero).withTimeout(1.0));

        NamedCommands.registerCommand("WaitForNote", new WaitShooterHasNote().withTimeout(1.0));

        NamedCommands.registerCommand("WaitNoteThenAutoAim",
                Commands.waitUntil(Subsystems.shooter::isNoteDetected).withTimeout(1.0)
                        .andThen(Subsystems.poseManager.getPoseCommand(PoseManager.Pose.ShooterAimVision)));

        // Common
        NamedCommands.registerCommand("StartIntake", Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Pickup));
        NamedCommands.registerCommand("FeedNote", new FeedNoteInAuto());


        // Tab
        NamedCommands.registerCommand("FeedNoteTab2", new FeedNoteInAutoTabShot2());
        NamedCommands.registerCommand("TabShot2Rotate", Tab.createTabShot2Rotate());
        NamedCommands.registerCommand("TabShot2", CommonCommands.DoTabShot2Command());
        NamedCommands.registerCommand("TabShot3", CommonCommands.DoShotCommand("TabShot3", Tab.ThirdShot));
        NamedCommands.registerCommand("TabShot4", CommonCommands.DoShotCommand("TabShot4", Tab.ForthShot));
        NamedCommands.registerCommand("TabShot5", CommonCommands.DoShotCommand("TabShot5", Tab.FifthShot));

        NamedCommands.registerCommand("TabQPose2", CommonCommands.queueTabShot2());
        NamedCommands.registerCommand("TabQPose4", CommonCommands.queueTabShot4());
        NamedCommands.registerCommand("TabFAS2", FeedAndShootAuto.createStandardShot(Tab.SecondShot));
        NamedCommands.registerCommand("TabFAS3", FeedAndShootAuto.createStandardShot(Tab.ThirdShot));
        NamedCommands.registerCommand("TabFAS4", FeedAndShootAuto.createStandardShot(Tab.ForthShot));
        // NamedCommands.registerCommand("TabFAS4", CommonCommands.feedAndShootAutoCmd(Tab.ForthShot));
        NamedCommands.registerCommand("TabFAS5", CommonCommands.feedAndShootAutoCmd(Tab.FifthShot));

        // DropShot
        NamedCommands.registerCommand("BloopShot", new BloopShot());

        // Blaze
        NamedCommands.registerCommand("BlazeQueuePivot", Subsystems.pivot.queueNextProfileCommand(Blaze.shootingPositionProfile));
        NamedCommands.registerCommand("BlazeShot", CommonCommands.DoBlazeShotCommand(Blaze.shootingPositionProfile));
        //SL1-SL3


    }

    /**
     * Registers the autonomous paths with the path registry
     */
    private void registerAutoPaths() {
        Tab.registerAutoPaths(pathRegistry);
        DropShot.registerAutoPaths(pathRegistry);
        Blaze.registerAutoPaths(pathRegistry);
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
        TabStraight,
        Blaze, 
        CrossWing,
        SL1toSL3
    }

}
