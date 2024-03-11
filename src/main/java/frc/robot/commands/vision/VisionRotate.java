package frc.robot.commands.vision;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.subsystems.util.BSLogger;
import frc.robot.subsystems.util.VisionAlignmentHelper;
import frc.robot.subsystems.vision.Pipeline;
import frc.robot.subsystems.vision.VisionTypes;

public class VisionRotate extends VisionCommand {
    private VisionAlignmentHelper helper = new VisionAlignmentHelper();

    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);



    private Pipeline visionPipeline = Pipeline.April;
    private int seenScans = 0;


    public VisionRotate() {
        this(null);
    }

    public VisionRotate(String limelightName) {
        super(limelightName);
        addRequirements(Subsystems.swerveSubsystem);
    }

    public VisionRotate withVisionPipeline(Pipeline pipeline) {
        this.visionPipeline = pipeline;
        return this;
    }

    public VisionRotate withTolerance(double tolerance) {
        this.helper.setTolerance(tolerance);
        return this;
    }

    @Override
    public void initialize() {
        var limelight = this.getLimelight();
        limelight.setCameraMode(VisionTypes.CameraMode.ImageProcessing);
        limelight.setPipelineIndex(this.visionPipeline.pipelineNumber);
        limelight.setLEDMode(VisionTypes.LEDMode.CurrentPipeline);
        BSLogger.log("VisionRotate", "starting with pipeline " + this.visionPipeline);
    }

    @Override
    public void execute() {
        VisionTypes.TargetInfo targetInfo = this.getLimelight().getTargetInfo();
        double horizontalComponent = Subsystems.rotationController.calculate(targetInfo.xOffset(), 0);
        double rotation = horizontalComponent * Constants.Swerve.kMaxAngularVelocity;

        seenScans = this.helper.inPosition() ? seenScans + 1 : 0;


        Subsystems.swerveSubsystem.setControl(
                drive.withVelocityX(0).withVelocityY(0)
                        .withRotationalRate(rotation));
    }


    @Override
    public boolean isFinished() {
        boolean finished = this.helper.inPosition() && seenScans > 5;
        if (finished) {
            BSLogger.log("VisionRotate", "finished within threshold");
        }
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (interrupted) {
            BSLogger.log("VisionRotate", "ended due to interrupt");
        }
    }

}
