package frc.robot.config;

import java.util.Map;
import java.util.Optional;

/**
 * Holds the robot configurations mapped by names to their respective options.
 * This allows for flexible and dynamic configuration management.
 */
public class RobotConfigurationOptions {
    private Map<String, RobotConfig> configurations;

    /**
     * Gets the map of robot configurations.
     * @return A map with configuration names as keys and RobotOption objects as values.
     */
    public Map<String, RobotConfig> getConfigurations() {
        return configurations;
    }

    /**
     * Sets the robot configurations map.
     * @param configurations A map with configuration names as keys and RobotOption objects as values.
     */
    public void setConfigurations(Map<String, RobotConfig> configurations) {
        this.configurations = configurations;
    }

    public Optional<RobotConfig> getConfiguration(String selectedConfiguration) {
        return Optional.ofNullable(this.configurations.get(selectedConfiguration));
    }
}
