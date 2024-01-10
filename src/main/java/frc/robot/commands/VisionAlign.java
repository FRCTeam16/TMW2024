package frc.robot.commands;


import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.subsystems.util.VisionAlignmentHelper;
import frc.robot.subsystems.vision.Limelight.CameraMode;
import frc.robot.subsystems.vision.Limelight.LEDMode;
import frc.robot.subsystems.vision.Pipeline;

public class VisionAlign extends Command {
    private VisionAlignmentHelper helper = new VisionAlignmentHelper();
    private double robotAngle = 180.0;
    private Pipeline visionPipeline = Pipeline.RetroHigh;
    private int seenScans = 0;

    private final SwerveRequest.FieldCentricFacingAngle alignDrive =
    new SwerveRequest.FieldCentricFacingAngle()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
       
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
        this.helper.overrideMaxSpeed(speedPercent);
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
        double currentAngle = Subsystems.swerveSubsystem.getPigeon2().getYaw().getValueAsDouble();
        double horizontalComponent = this.helper.calculate();   // pid adjustment

        // assuming field relative
        double direction = (Math.abs(currentAngle) < 90) ? 1 : -1;
        double velocity = direction * horizontalComponent * Constants.Swerve.kMaxSpeedMetersPerSecond;

        // Track how long we've been in position
        if (this.helper.inPosition()) {
           seenScans++;
        } else {
            seenScans = 0;
        }

        Subsystems.swerveSubsystem.applyRequest( () ->
            alignDrive.withVelocityX(0)
            .withVelocityY(velocity)
            .withTargetDirection(Rotation2d.fromDegrees(robotAngle)));
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
