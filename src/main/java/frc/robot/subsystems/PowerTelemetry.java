package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems;

public class PowerTelemetry {
    private final PowerDistribution pdh = new PowerDistribution();
    private boolean logCANDevices = false;
    public void periodic() {
        SmartDashboard.putNumber("Power/Voltage", RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("Power/Current", RobotController.getInputCurrent());

        if (logCANDevices) {
            logPDH();
            logSwerve();
        }
    }

    public PowerTelemetry withLogCANDevices(boolean logCANDevices) {
        this.logCANDevices = logCANDevices;
        return this;
    }

    private static void logSwerve() {
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

    private void logPDH() {
        SmartDashboard.putNumber("Power/PDH/Voltage", pdh.getVoltage());
        SmartDashboard.putNumber("Power/PDH/TotalCurrent", pdh.getTotalCurrent());
    }
}
