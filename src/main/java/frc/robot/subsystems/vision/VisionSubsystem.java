package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.vision.VisionTypes.TargetInfo;

import java.util.HashMap;
import java.util.Map;

public class VisionSubsystem extends SubsystemBase implements Lifecycle {
    private final Map<String, Limelight> limelightLookup = new HashMap<>();
    private final Limelight defaultLimelight;

    public VisionSubsystem(Iterable<Limelight> limelights) {
        Limelight tmpLimelight = null;
        for (Limelight limelight : limelights) {
            limelightLookup.put(limelight.getName(), limelight);
            if (tmpLimelight == null) {
                tmpLimelight = limelight;
            }
        }
        if (tmpLimelight != null) {
            this.defaultLimelight = tmpLimelight;
        } else {
            throw new IllegalArgumentException("At least one Limelight must be specified");
        }
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    public Limelight getDefaultLimelight() {
        return defaultLimelight;
    }

    public Limelight getLimelightByName(String name) {
        Limelight limelight = limelightLookup.get(name);
        if (limelight != null) {
            return limelight;
        } else {
            String message = "ERROR: No limelight named %s was added to the VisionSubsystem".formatted(name);
            DataLogManager.log(message);
            throw new IllegalArgumentException(message);
        }
    }

    public boolean hasTarget(){
        TargetInfo info = this.getDefaultLimelight().getTargetInfo();
        return info.hasTarget() && Math.abs(info.xOffset()) < 2.5;
    }

}
