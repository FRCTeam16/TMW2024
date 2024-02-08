package frc.robot.config;

/**
 * Represents specific configuration options for the robot, such as motor CAN IDs.
 */
public record RobotConfig (
        SwerveConfig swerve,
        int pigeonId) {

    static RobotConfig createDefaultConfig() {
        return new RobotConfig(
                new SwerveConfig(
                        new SwerveModuleConfig(1, 2, 1),
                        new SwerveModuleConfig(3, 4, 2),
                        new SwerveModuleConfig(5, 6, 3),
                        new SwerveModuleConfig(7, 8, 9)
                ),
                1
        );
    }
}

