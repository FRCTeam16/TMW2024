package frc.robot.config;

public record EnableSubsystemsConfig(
        boolean swerve,
        boolean vision,
        boolean led,
        boolean intake,
        boolean shooter,
        boolean pivot,
        boolean climber,
        boolean trap) {

    public static EnableSubsystemsConfig createDefaultConfig() {
        return new EnableSubsystemsConfig(
                true,
                true,
                true,
                true,
                true,
                true,
                true,
                true
        );
    }
}
