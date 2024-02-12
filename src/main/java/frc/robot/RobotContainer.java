package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AutoManager;
import frc.robot.commands.RunWithDisabledInstantCommand;
import frc.robot.commands.vision.VisionAlign;
import frc.robot.commands.ZeroAndSetOffsetCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.Pivot;

import java.util.Objects;

public class RobotContainer {
    private static final double MaxSpeed = Constants.Swerve.kMaxSpeedMetersPerSecond;
    private static final double MaxAngularRate = Constants.Swerve.kMaxAngularVelocity;

    private final Joystick left = new Joystick(0);
    private final Joystick right = new Joystick(1);
    private final CommandXboxController xboxController = new CommandXboxController(2);
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

    @SuppressWarnings("unused")
    private final Subsystems subsystems = Subsystems.getInstance();
    private final AutoManager autoManager = new AutoManager();

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveStateTelemetry swerveStateTelemetry = new SwerveStateTelemetry(MaxSpeed);

    //
    // Left Joystick
    //
    private final JoystickButton intake = new JoystickButton(left, 1);
    private final JoystickButton eject = new JoystickButton(left, 2);
    private final JoystickButton slowIntake = new JoystickButton(left, 4);


    //
    // Right Joystick
    //
    private final Trigger feed = new Trigger(right::getTrigger);
    private final JoystickButton runVisionAlignAngle = new JoystickButton(right, 2);

    //
    // Controller
    //



    public RobotContainer() {
        configureBindings();
        configureDashboardButtons();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-right.getY() * MaxSpeed)
                        .withVelocityY(-right.getX() * MaxSpeed)
                        .withRotationalRate(-left.getX() * MaxAngularRate)));


        //
        // Debug/Development
        //

//        xboxController.y().whileTrue(drivetrain.applyRequest(() -> brake));
//        xboxController.b().whileTrue(drivetrain
//                .applyRequest(
//                        () -> point.withModuleDirection(new Rotation2d(-xboxController.getLeftY(), -xboxController.getLeftX()))));

        xboxController.rightBumper().onTrue(Commands.runOnce(Subsystems.pivot::openLoopUp)).onFalse((Commands.runOnce(Subsystems.pivot::holdPosition)));
        xboxController.leftBumper().onTrue(Commands.runOnce(Subsystems.pivot::openLoopDown)).onFalse((Commands.runOnce(Subsystems.pivot::holdPosition)));
        xboxController.y().onTrue(Commands.runOnce(() -> Subsystems.pivot.setPivotPosition(Pivot.PivotPosition.StartingPosition)));
        xboxController.x().onTrue(Commands.runOnce(() -> Subsystems.pivot.setPivotPosition(Pivot.PivotPosition.FeedPosition)));
        xboxController.a().onTrue(Commands.runOnce(() -> Subsystems.pivot.setPivotPosition(Pivot.PivotPosition.Up)));

        runVisionAlignAngle.whileTrue(new VisionAlign().withRobotAngle(90.0));
        //        lockAngle1.onTrue(new RotateToAngle(-60));
        //        lockAngle2.onTrue(new RotateToAngle(0));



        //
        // Intake Subsystem
        //
        intake.onTrue(Commands.runOnce(Subsystems.intake::runIntakeFast)).onFalse(Commands.runOnce(Subsystems.intake::stopIntake));
        slowIntake.onTrue(Commands.runOnce(Subsystems.intake::runIntakeSlow)).onFalse(Commands.runOnce(Subsystems.intake::stopIntake));
        eject.onTrue(Commands.runOnce(Subsystems.intake::runIntakeEject)).onFalse(Commands.runOnce(Subsystems.intake::stopIntake));


        //
        // Shooter
        //
        SmartDashboard.putData("Start Shooter", Commands.runOnce(Subsystems.shooter::runShooter));
        SmartDashboard.putData("Stop Shooter", Commands.runOnce(Subsystems.shooter::stopShooter));
        feed.onTrue(Commands.runOnce(Subsystems.shooter::runFeeder))
                .onFalse(Commands.runOnce(Subsystems.shooter::stopFeeder));


        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(3,3), Rotation2d.fromDegrees(0)));
        }
        drivetrain.registerTelemetry(swerveStateTelemetry::telemeterize);

    }

    private void configureDashboardButtons() {
        SmartDashboard.putData("Tare Odometry", new RunWithDisabledInstantCommand(() -> Subsystems.swerveSubsystem.tareEverything()));
        SmartDashboard.putData("Zero Gyro", new ZeroAndSetOffsetCommand(0).ignoringDisable(true));
    }

    public Command getAutonomousCommand() {
        return autoManager.getSelectedCommand();
    }

    public void teleopInit() {
        Subsystems.lifecycleSubsystems.stream().filter(Objects::nonNull).forEach(Lifecycle::teleopInit);
    }

    public void autoInit() {
        Subsystems.lifecycleSubsystems.stream().filter(Objects::nonNull).forEach(Lifecycle::autoInit);
    }

    public void robotPeriodic() {

        // FIXME: Basic tracking is kind of going, but very noisy and we need to figure out yaw handling
    /*
    var info = Subsystems.visionSubsystem.getVisionInfo();
    System.out.println("VP: " + info.botPose);
    System.out.println("HT: " + info.hasTarget);
    if (info.hasTarget && info.botPose != null) {
      System.out.println(info.botPose);
      double captureTime = Timer.getFPGATimestamp() - (info.poseLatency/1000.0);
      Subsystems.swerveSubsystem.addVisionMeasurement(info.botPose, captureTime);
    }
    */

        // Debug telemetry
        SmartDashboard.putNumber("Yaw/PigeonAngle", Subsystems.swerveSubsystem.getPigeon2().getAngle());
        SmartDashboard.putNumber("Yaw/PigeonYaw", Subsystems.swerveSubsystem.getPigeon2().getYaw().getValueAsDouble());
        SmartDashboard.putNumber("Yaw/SwerveYaw", Subsystems.swerveSubsystem.getYaw());
    }

}
