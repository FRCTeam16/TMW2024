package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.MedianFilter;
import frc.robot.subsystems.vision.VisionTypes.TargetInfo;

public class HasTargetFilter {
    private MedianFilter targetFilter = new MedianFilter(5);

    public boolean calculate(TargetInfo info) {
        double result = targetFilter.calculate(info.hasTarget() ? 1 : 0);
        return result > 0.5;
    }
    
}
