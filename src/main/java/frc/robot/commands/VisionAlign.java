package frc.robot.commands;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.subsystems.RotationController;
import frc.robot.subsystems.util.VisionAlignmentHelper;
import frc.robot.subsystems.vision.Limelight.CameraMode;
import frc.robot.subsystems.vision.Limelight.LEDMode;
import frc.robot.subsystems.vision.Pipeline;

public class VisionAlign extends Command {
    private VisionAlignmentHelper helper = new VisionAlignmentHelper();
    private double robotAngle = 180.0;
    private Pipeline visionPipeline = Pipeline.RetroHigh;
    private double robotSpeed = 0.25;
    private int seenScans = 0;

    public VisionAlign() {
        addRequirements(Subsystems.swerveSubsystem);
    }

    public VisionAlign withVisionPipeline(Pipeline pipeline) {
        this.visionPipeline = pipeline;
        return this;
    }

    public VisionAlign withRobotAngle(double angle) {
        this.robotAngle = angle;
        return this;
    }

    public VisionAlign withTolerance(double tolerance) {
        this.helper.setTolerance(tolerance);
        return this;
    }

    public VisionAlign withRobotSpeed(double speedPercent) {
        this.robotSpeed = speedPercent;
        return this;
    }
    
    @Override
    public void initialize() {
        Subsystems.visionSubsystem.getLimelight().setCameraMode(CameraMode.ImageProcessing);
        Subsystems.visionSubsystem.selectPipelineSync(this.visionPipeline);
        // Subsystems.visionSubsystem.getLimelight().setCurrentPipeline(Pipeline.RetroHigh.pipelineNumber);
        Subsystems.visionSubsystem.getLimelight().setLEDMode(LEDMode.CurrentPipeline);
    }


    @Override
    public void execute() {
        // FIXME: Needs to use new swerve system

        // RotationController controller = Subsystems.swerveSubsystem.getRotationController();
        // double currentAngle = Subsystems.swerveSubsystem.getYaw().getDegrees();
        // double twist = controller.calculate(currentAngle, robotAngle);
        // double horizontalComponent = this.helper.calculate();

        // // assuming field relative
        // double direction = (Math.abs(Subsystems.swerveSubsystem.getYaw().getDegrees()) < 90) ? 1 : -1;
        // Translation2d translation = new Translation2d(0, direction * horizontalComponent);

        // if (this.helper.inPosition()) {
        //    seenScans++;
        // } else {
        //     seenScans = 0;
        // }

        // Subsystems.swerveSubsystem.drive(
        //     translation.times(robotSpeed * Constants.Swerve.kMaxSpeedMetersPerSecond), 
        //     Math.toRadians(twist), 
        //     true, 
        //     true);
    }

    @Override
    public boolean isFinished() {
        boolean finished = this.helper.inPosition() && seenScans > 5;
        if (finished) {
            System.out.println("*** VisionAlign finished in position ***");
        }
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (interrupted) {
            System.out.println("~~~ VisionAlignment ended due to interrupt");
        } 
    }
}
