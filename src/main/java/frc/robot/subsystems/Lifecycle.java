package frc.robot.subsystems;

/** Lifecycle method hooks for subsystems. */
public interface Lifecycle {
    default void teleopInit() {}

    default void autoInit() {}
}
