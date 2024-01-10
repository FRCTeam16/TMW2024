package frc.robot.subsystems.gyro;
/*
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
*/

public class PigeonGyro /*implements BSGyro*/ {
    /* 
    // private final Pigeon2 m_pigeon;
    private final WPI_Pigeon2 m_pigeon;
    private double offset = 0.0;

    public PigeonGyro(int CAN_ID) {
        m_pigeon = new WPI_Pigeon2(CAN_ID);
        m_pigeon.configEnableCompass(false);
        // m_pigeon.configMountPoseYaw(180);   // TODO: This is 2023 only
        // var errorCode = m_pigeon.configFactoryDefault();
    }

    @Override
    public Rotation2d getGyroscopeRotation() {
        double degrees = m_pigeon.getYaw() % 360.0;
        degrees += offset;
        if (degrees < -180.0) {
            degrees += 360;
        } else if (degrees > 180.0) {
            degrees -= 360.0;
        }
        SmartDashboard.putNumber("PigeonGyro/Base", m_pigeon.getYaw());
        SmartDashboard.putNumber("PigeonGyro/Adjusted", degrees+offset);

        return Rotation2d.fromDegrees(degrees);
    }

    @Override
    public void zeroGyroscope() {
        m_pigeon.setYaw(0.0);
        m_pigeon.setAccumZAngle(0);
    }

    @Override
    public void setGyroOffset(double offsetDegrees) {
        //m_pigeon.setYaw(offsetDegrees);
        this.offset = offsetDegrees;
        SmartDashboard.putNumber("PigeonGyro/Offset", offsetDegrees);
    }

    @Override
    public double getPitch() {
        return m_pigeon.getPitch();
    }

    @Override
    public double getRoll() {
        return m_pigeon.getRoll();
    }
    */
}