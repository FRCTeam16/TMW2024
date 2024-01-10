package frc.robot;

public final class Constants {

    public static final class OI {
        public static final double stickDeadband = 0.05;
    }

    // TODO: speeds need tuning
    public static final class Swerve {
        public static final double kMaxSpeedMetersPerSecond = 6.0;
        public static final double kMaxAngularVelocity = 3 * Math.PI;
    }
    
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 6.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 6.0;

    }
}
