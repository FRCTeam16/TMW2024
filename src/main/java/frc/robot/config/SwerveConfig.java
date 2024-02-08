package frc.robot.config;

public record SwerveConfig(
        SwerveModuleConfig fl,
        SwerveModuleConfig fr,
        SwerveModuleConfig rl,
        SwerveModuleConfig rr) {
}
