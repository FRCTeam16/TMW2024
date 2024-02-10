package frc.robot.config;

public record SwerveConfig(
        double speedAt12v,
        double coupleRatio,
        double driveGearRatio,
        double steerGearRatio,
        SwerveModuleConfig fl,
        SwerveModuleConfig fr,
        SwerveModuleConfig rl,
        SwerveModuleConfig rr) {
}
