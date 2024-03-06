package frc.robot.commands.auto;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.subsystems.RotationController;
import frc.robot.subsystems.util.BSLogger;
import frc.robot.subsystems.util.GameInfo;

public class RotateToAngle extends Command {
    private final double baseTargetAngleDegrees;
    private final int scansToHold = 5;
    private RotationController rotationController = Subsystems.rotationController;
    private double targetAngleDegrees;
    private Double overrideClamp;
    private double lastScanTime = 0;
    private SwerveRequest.FieldCentric rotate =
            new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private int scanCount = 0;

    private boolean mirror = false;

    /**
     * Rotates the robot to a specific angle, assumes field mirror will be used
     *
     * @param targetAngleDegrees
     */

    public RotateToAngle(double targetAngleDegrees) {
        this(targetAngleDegrees, true);
    }

    public RotateToAngle(double targetAngleDegrees, boolean mirror) {
        this.mirror = mirror;
        this.baseTargetAngleDegrees = targetAngleDegrees;
        addRequirements(Subsystems.swerveSubsystem);
    }

    public RotateToAngle withOverrideRadianClamp(double clamp) {
        this.overrideClamp = clamp;
        return this;
    }

    public RotateToAngle withThreshold(double threshold) {
        rotationController.setTolerance(threshold);
        return this;
    }

    @Override
    public void initialize() {
        this.lastScanTime = Timer.getFPGATimestamp();

        // Determine true targetAngleDegrees based on mirror/flip state and request
        targetAngleDegrees = baseTargetAngleDegrees;
        if (mirror && GameInfo.isRedAlliance()) {
            Rotation2d rotation = GeometryUtil.flipFieldRotation(Rotation2d.fromDegrees(baseTargetAngleDegrees));
            targetAngleDegrees = rotation.getDegrees();
            BSLogger.log("RotateToAngle",
                    "flipping rotation from %.2f to %.2f".formatted(baseTargetAngleDegrees, targetAngleDegrees));
        }
        rotationController.setSetpoint(targetAngleDegrees);
    }

    @Override
    public void execute() {

        // double clamp = SmartDashboard.getNumber("AutoRotationClamp", 30);
        // if (this.overrideClamp != null) {
        //     clamp = overrideClamp;
        // }

        double twist = rotationController.calculate(
                Subsystems.swerveSubsystem.getYaw(),
                this.targetAngleDegrees);


        // double clampedTwist = MathUtil.clamp(Math.toRadians(twist), -clamp, clamp);
        // twist = RotationController.clampToDPS(0.2);
        final double twistRate = twist * Constants.Swerve.kMaxAngularVelocity;

        Subsystems.swerveSubsystem.setControl(rotate.withRotationalRate(twistRate));

        if (Constants.Dashboard.UseSendables) {
            SmartDashboard.putData("RotateToAngle/Controller", rotationController);
            SmartDashboard.putNumber("RotateToAngle/Error", rotationController.getPositionError());
            SmartDashboard.putNumber("RotateToAngle/Target", rotationController.getSetpoint());
            if (Constants.Dashboard.ConfigurationMode) {
                SmartDashboard.putNumber("RotateToAngle/Twist", twist);
                SmartDashboard.putNumber("RotateToAngle/TwistRate", twistRate);
            }
        }
    }

    @Override
    public boolean isFinished() {
        if (rotationController.atSetpoint()) {
            if (++scanCount > scansToHold) {
                return true;
            }
        } else {
            scanCount = 0;
        }
        return rotationController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        rotationController.resetTolerance();
        Subsystems.swerveSubsystem.setControl(rotate.withRotationalRate(0));
    }

}
