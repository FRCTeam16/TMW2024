package frc.robot.subsystems.gyro;

// import com.kauailabs.navx.frc.AHRS;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.I2C;

/** NavX gyro stub
 * @hidden
 * Needs translation before use 
 */
public class NavXGyro /*implements BSGyro*/ {
    /* 
    private final AHRS m_navx;

    public NavXGyro() {
        m_navx = new AHRS(I2C.Port.kMXP, (byte) 200); // NavX
    }

    @Override
    public Rotation2d getGyroscopeRotation() {
        if (m_navx.isMagnetometerCalibrated()) {
        // We will only get valid fused headings if the magnetometer is calibrated
        return Rotation2d.fromDegrees(m_navx.getFusedHeading());
        }
        
        // We have to invert the angle of the NavX so that rotating the robot
        // counter-clockwise makes the angle increase.
        return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
    }

    @Override
    public void zeroGyroscope() {
        m_navx.zeroYaw();
    }

    @Override
    public void setGyroOffset(double offsetDegrees) {
        // TODO: needs testing
        m_navx.setAngleAdjustment(offsetDegrees);
    }

    @Override
    public double getPitch() {
        return m_navx.getPitch();
    }

    @Override
    public double getRoll() {
        return m_navx.getRoll();
    }
    */
}
