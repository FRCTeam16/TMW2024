package frc.robot.auto;

import java.util.HashMap;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.auto.strategies.DebugAuto;
import frc.robot.auto.strategies.DebugPathAuto;
import frc.robot.commands.auto.RotateToAngle;

import com.pathplanner.lib.auto.NamedCommands;


public class AutoManager {

    public enum AutoStrategies {
        DebugAuto,
        DebugTestPath,
        DebugTestMultipath,
        DebugCurly,
        DebugWavy,
        DebugTests2
    }

    private final SendableChooser<AutoStrategies> chooser = new SendableChooser<>();
    private final HashMap<AutoStrategies, Supplier<Command>> strategyLookup = new HashMap<>();

    public AutoManager() {
        registerCommmands();
        registerStrategy("Debug Auto", AutoStrategies.DebugAuto, DebugAuto::new);
        registerStrategy("Test Path", AutoStrategies.DebugTestPath, () -> new DebugPathAuto("TestStageLeft"));
        registerStrategy("Test MultiPath", AutoStrategies.DebugTestMultipath, () -> new DebugPathAuto("Multipoint Test"));
        registerStrategy("Test Curly", AutoStrategies.DebugCurly, () -> new DebugPathAuto("TestCurly"));
        registerStrategy("Test Wavy", AutoStrategies.DebugWavy, () -> new DebugPathAuto("TestWave"));
        registerStrategy("Tests2", AutoStrategies.DebugTests2, () -> new DebugPathAuto("Tests2"));
        
        
        // Send selector Dashboard. If it doesn't show in SD, you may need to change the
        // name here.
        SmartDashboard.putData("Auto Selector", chooser);
    }

    private void registerCommmands(){
        NamedCommands.registerCommand("rotate45", new RotateToAngle(-45));
    }

    private void registerStrategy(String displayName, AutoStrategies strategyEnum, Supplier<Command> strategy) {
        registerStrategy(displayName, strategyEnum, strategy, false);
    }

    private void registerStrategy(String displayName, AutoStrategies strategyEnum, Supplier<Command> strategy,
            boolean isDefault) {
        if (isDefault) {
            chooser.setDefaultOption(displayName, strategyEnum);
        } else {
            chooser.addOption(displayName, strategyEnum);
        }
        strategyLookup.put(strategyEnum, strategy);
    }

    /**
     * Returns the currently selected autonomous strategy
     * 
     * @return the currently selected autonomous strategy
     */
    public Command getSelectedCommand() {
        Command selected = null;

        if (strategyLookup.containsKey(chooser.getSelected())) {
            selected = strategyLookup.get(chooser.getSelected()).get();
        } else {
            // Missing autonomous key in lookup map
            selected = new InstantCommand(
                    () -> System.out.println("ERROR: Could not find requested auto: " + chooser.getSelected()));
        }
        return selected;
    }

    public void showSelectedAuto() {
        var selected = chooser.getSelected();
        SmartDashboard.putString("Selected Auto", (selected != null) ? selected.name() : "Unknown");
    }

}
