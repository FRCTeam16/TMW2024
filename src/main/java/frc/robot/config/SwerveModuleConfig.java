package frc.robot.config;

public record SwerveModuleConfig(
        int driveId,
        int steerId,
        int encoderId,
        double encoderOffset
) {
}
