package frc.robot.config;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

public class TestRobotConfiguration {
    public static final String CONFIG_FILE = "src/main/deploy/config/robot-configs.json";

    @Test
    void testDefault() {
        RobotConfiguration.initialize();
        assertNotNull(RobotConfiguration.config);
        assertEquals(1, RobotConfiguration.config.pigeonId());
        assertEquals(2, RobotConfiguration.config.swerve().fl().steerId());
    }

    @Test
    void testReadCompetitionConfig() {
        try {
            RobotConfiguration.getEnvironmentVariables().put(RobotConfiguration.SELECTED_CONFIG_ENV, "competition");
            RobotConfiguration.initialize(CONFIG_FILE);
            assertNotNull(RobotConfiguration.config);
        } finally {
            RobotConfiguration.getEnvironmentVariables().remove(RobotConfiguration.SELECTED_CONFIG_ENV);
        }
    }

    @Test
    void testReadUnknownConfig() {
        try {
            RobotConfiguration.getEnvironmentVariables().put(RobotConfiguration.SELECTED_CONFIG_ENV, "unknown");
            RobotConfiguration.initialize(CONFIG_FILE);
            assertNotNull(RobotConfiguration.config);
            assertEquals(7, RobotConfiguration.config.swerve().rr().driveId());
        } finally {
            RobotConfiguration.getEnvironmentVariables().remove(RobotConfiguration.SELECTED_CONFIG_ENV);
        }
    }
}
