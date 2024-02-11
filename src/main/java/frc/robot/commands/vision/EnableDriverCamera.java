package frc.robot.commands.vision;

import frc.robot.subsystems.vision.VisionTypes;

public class EnableDriverCamera extends VisionCommand {

    @Override
    public void initialize() {
        this.getLimelight().setLEDMode(VisionTypes.LEDMode.ForceOff);
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
