package frc.robot.auto;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DataLogManager;

/**
 * Registry for paths to be used in autonomous mode
 * The swerve base and AutoBuilder must be instantiated before this class is
 * used
 */
public class PathRegistry {
    private final Map<String, PathPlannerAuto> pathMap;

    public PathRegistry() {
        pathMap = new HashMap<>();
    }

    public void registerPath(String pathName) {
        if (!pathMap.containsKey(pathName)) {
            DataLogManager.log("[PathRegistry] Registering path " + pathName);
            try {
                pathMap.put(pathName, new PathPlannerAuto(pathName));
            } catch (Exception e) {
                String message = "[PathRegistry]Error while trying to register an auto path named %s: %s"
                        .formatted(pathName, e);
                throw new RuntimeException(message);
            }
        }
    }

    public PathPlannerAuto getPath(String pathName) {
        if (!pathMap.containsKey(pathName)) {
            DataLogManager.log(
                    "[PathRegistry] Path " + pathName + " was not registered, please register for performance reasons");
            registerPath(pathName);
        }
        return pathMap.get(pathName);
    }

    public boolean hasPath(String pathName) {
        return pathMap.containsKey(pathName);
    }

    public void registerPaths(String... paths) {
        for (String path : paths) {
            registerPath(path);
        }
    }
}