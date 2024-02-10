package frc.robot;

import frc.robot.generated.TunerConstants;

public final class Constants {

    public static final class OI {
        public static final double stickDeadband = 0.05;
    }

    // TODO: speeds need tuning
    public static final class Swerve {
        public static final double kMaxSpeedMetersPerSecond = TunerConstants.kSpeedAt12VoltsMps;
        public static final double kMaxAngularVelocity = 4 * Math.PI;
    }
    
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = TunerConstants.kSpeedAt12VoltsMps;
        public static final double kMaxAccelerationMetersPerSecondSquared = 6.0;

    }
}
