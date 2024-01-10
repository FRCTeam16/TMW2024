package frc.robot.subsystems.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Gyroscope
 * 
 * The important thing about how you configure your gyroscope is that rotating
 * the robot counter-clockwise should cause the angle reading to increase until
 * it wraps back over to zero.
 * 
 */
public interface BSGyro {

    /**
     * Gets the current gyroscopic rotation.
     * 
     * @return
     */
    Rotation2d getGyroscopeRotation();


    double getPitch();
    double getRoll();

    /*
     * Sets the gyroscope angle to zero. This can be used to set the direction the
     * robot is currently facing to the 'forwards' direction.
     */
    void zeroGyroscope();

    public void setGyroOffset(double offsetDegrees);
}
