package frc.robot.commands.vision;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.subsystems.util.VisionAlignmentHelper;
import frc.robot.subsystems.vision.Pipeline;
import frc.robot.subsystems.vision.VisionTypes;

public class VisionAlign extends VisionCommand {
    private VisionAlignmentHelper helper = new VisionAlignmentHelper();
    private Optional<Double> robotAngle = Optional.empty();
    private Pipeline visionPipeline = Pipeline.April;
    private int seenScans = 0;

    private final SwerveRequest.RobotCentric alignRobotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    DoubleLogEntry pidLog;
    DoubleLogEntry velocityLog;
    DoubleLogEntry twistLog;

    public VisionAlign(String limelightName) {
        super(limelightName);
        addRequirements(Subsystems.swerveSubsystem);

        DataLog log = DataLogManager.getLog();
        pidLog = new DoubleLogEntry(log, "VisionAlign/PID");
        velocityLog = new DoubleLogEntry(log, "VisionAlign/Velocity");
        twistLog = new DoubleLogEntry(log, "VisionAlign/Twist");
    }

    public VisionAlign() {
        this(null);
    }

    public VisionAlign withVisionPipeline(Pipeline pipeline) {
        this.visionPipeline = pipeline;
        return this;
    }

    public VisionAlign withRobotAngle(double angle) {
        this.robotAngle = Optional.of(angle);
        return this;
    }

    public VisionAlign withTolerance(double tolerance) {
        this.helper.setTolerance(tolerance);
        return this;
    }

    public VisionAlign withRobotSpeed(double speedPercent) {
        this.helper.overrideMaxSpeed(speedPercent);
        return this;
    }

    @Override
    public void initialize() {
        var limelight = this.getLimelight();
        limelight.setCameraMode(VisionTypes.CameraMode.ImageProcessing);
        limelight.setPipelineIndex(this.visionPipeline.pipelineNumber);
        limelight.setLEDMode(VisionTypes.LEDMode.CurrentPipeline);
        DataLogManager.log("[VisionAlign] starting");
    }

    @Override
    public void execute() {
        double horizontalComponent = this.helper.calculate(this.getLimelight().getTargetInfo()); // pid adjustment for vision offset
        double velocity = horizontalComponent * Constants.Swerve.kMaxSpeedMetersPerSecond;

        seenScans = this.helper.inPosition() ? seenScans + 1 : 0;

        // Telemetry
        pidLog.append(horizontalComponent);
        velocityLog.append(velocity);

        if (robotAngle.isEmpty()) {
            Subsystems.swerveSubsystem.setControl(alignRobotCentricDrive.withVelocityX(0)
                    .withVelocityY(velocity));
        } else {
            double twistRate = Subsystems.rotationController.calculate(Subsystems.swerveSubsystem.getYaw(),
                    robotAngle.get()) * Constants.Swerve.kMaxAngularVelocity;

            twistLog.append(twistRate);
            Subsystems.swerveSubsystem.setControl(alignRobotCentricDrive
                    .withVelocityX(0)
                    .withVelocityY(velocity)
                    .withRotationalRate(twistRate));
        }

    }

    @Override
    public boolean isFinished() {
        boolean finished = this.helper.inPosition() && seenScans > 5;
        if (finished) {
            DataLogManager.log("[VisionAlign] finished within threshold");
        }
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (interrupted) {
            DataLogManager.log("[VisionAlign] ended due to interrupt");
        }
    }
}
