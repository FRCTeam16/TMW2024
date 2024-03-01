package frc.robot;

import frc.robot.generated.TunerConstants;

public final class Constants {


    public static final class Dashboard {
        // Whether to use the sendable system for dashboard updates
        public static final boolean UseSendables = true;
        // Whether to use the sendable system for real-time tuning
        public static final boolean ConfigurationMode = true;
    }

    public static final class OI {
        public static final double stickDeadband = 0.05;
    }

    public static final class Swerve {
        public static final double kMaxSpeedMetersPerSecond = TunerConstants.kSpeedAt12VoltsMps;
        public static final double kMaxAngularVelocity = 4 * Math.PI;
    }
    
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = TunerConstants.kSpeedAt12VoltsMps;
        public static final double kMaxAccelerationMetersPerSecondSquared = 6.0;

    }
}
