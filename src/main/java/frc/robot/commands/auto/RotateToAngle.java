package frc.robot.commands.auto;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Subsystems;
// import frc.robot.subsystems.RotationController;

public class RotateToAngle extends Command {
    // private RotationController rotationController = Subsystems.swerveSubsystem.getRotationController();
    // private final double targetAngleDegrees;
    // private Double overrideClamp;
    // private double lastScanTime = 0;

    // private final int scansToHold = 5;
    // private int scanCount = 0;

    // public RotateToAngle(double targetAngleDegrees) {
    //     this.targetAngleDegrees = targetAngleDegrees;
    //     rotationController.setSetpoint(targetAngleDegrees);
    //     addRequirements(Subsystems.swerveSubsystem);
    // }

    // public RotateToAngle withOverrideRadianClamp(double clamp) {
    //     this.overrideClamp = clamp;
    //     return this;
    // }

    // public RotateToAngle withThreshold(double threshold) {
    //     rotationController.setTolerance(threshold);
    //     return this;
    // }

    // @Override
    // public void initialize() {
    //     this.lastScanTime = Timer.getFPGATimestamp();
    // }

    // @Override
    // public void execute() {
    //     double clamp = SmartDashboard.getNumber("AutoRotationClamp", 30);
    //     if (this.overrideClamp != null) {
    //         clamp = overrideClamp;
    //     }
    //     double twist = rotationController.calculate(
    //             Subsystems.swerveSubsystem.getYaw().getDegrees(),
    //             this.targetAngleDegrees);

    //     double clampedTwist = MathUtil.clamp(Math.toRadians(twist), -clamp, clamp);
    //     Subsystems.swerveSubsystem.drive(
    //             new Translation2d(0, 0),
    //             clampedTwist,
    //             true,
    //             true);

    //     double now = Timer.getFPGATimestamp();
    //     ;
    //     SmartDashboard.putNumber("RotateToAngle Scan Time", (now - lastScanTime));
    //     lastScanTime = now;
    //     SmartDashboard.putNumber("RotateToAngleError", rotationController.getPositionError());
    // }

    // @Override
    // public boolean isFinished() {
    //     // if (rotationController.atSetpoint()) {
    //     // if (++scanCount > scansToHold) {
    //     // return true;
    //     // }
    //     // } else {
    //     // scanCount = 0;
    //     // }
    //     return rotationController.atSetpoint();
    // }

    // @Override
    // public void end(boolean interrupted) {
    //     rotationController.resetTolerance();
    // }

}
