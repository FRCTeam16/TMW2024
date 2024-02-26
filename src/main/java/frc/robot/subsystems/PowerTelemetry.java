package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems;

public class PowerTelemetry {
    private final PowerDistribution pdh = new PowerDistribution();
    public void periodic() {
        SmartDashboard.putNumber("Power/Voltage", pdh.getVoltage());
        SmartDashboard.putNumber("Power/TotalCurrent", pdh.getTotalCurrent());

        // Swerve
        for (int i=0;i<4;i++) {
            var module = Subsystems.swerveSubsystem.getModule(i);
            SmartDashboard.putNumber("Power/Swerve[" + i + "]/Drive/Voltage",
                    module.getDriveMotor().getMotorVoltage().getValue());
            SmartDashboard.putNumber("Power/Swerve[" + i + "]/Drive/Stator",
                    module.getDriveMotor().getStatorCurrent().getValue());
            SmartDashboard.putNumber("Power/Swerve[" + i + "]/Drive/Supply",
                    module.getDriveMotor().getSupplyCurrent().getValue());
        }
    }
}
