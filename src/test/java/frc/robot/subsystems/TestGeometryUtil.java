package frc.robot.subsystems;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class TestGeometryUtil {

    @Test
    public void testFlipFieldRotation() {
        // Prepare data
        Rotation2d angle = Rotation2d.fromDegrees(0);
        // Test method
        Rotation2d result = GeometryUtil.flipFieldRotation(angle);
        // Assert that the result is correct
        Assertions.assertEquals(180, result.getDegrees(), "The result should be 270");
    }

    @Test
    public void testFlipFieldPose() {
        // Prepare data
        Pose2d pose2d = new Pose2d(
                1.84, 7.44, new Rotation2d(0));
        // Test method
        Pose2d result = GeometryUtil.flipFieldPose(pose2d);
        Pose2d expected = new Pose2d(
                14.70, 7.44, Rotation2d.fromDegrees(180));
        // Assert that the result is correct
        Assertions.assertEquals(expected, result, "The result should be " + expected);
    }
}
