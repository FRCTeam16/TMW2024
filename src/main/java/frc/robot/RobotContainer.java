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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AutoManager;
import frc.robot.commands.RunWithDisabledInstantCommand;
import frc.robot.commands.ZeroAndSetOffsetCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.RotationController;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pose.PoseManager;
import frc.robot.subsystems.trap.TrapExtender;
import frc.robot.subsystems.vision.VisionTypes;

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
            .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.01) // Add a 10% deadband
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
    //private final JoystickButton feedNote = new JoystickButton(right, 3);
    private final JoystickButton ampAim = new JoystickButton(right, 4);

    //
    // Controller
    //
    private final RotationController alignController = new RotationController(0.02, 0, 0);
    //
    MusicController music = new MusicController();
    private boolean useVisionAlignment = false;

    public RobotContainer() {
        configureBindings();
        configureDashboardButtons();
        alignController.setTolerance(0.05);
        SmartDashboard.putData("AlignPID", alignController);

    }

    private double supplySwerveRotate() {
        final double DEADBAND = 0.05;
        if (!useVisionAlignment) {
//            return OIUtil.deadband(-left.getX(), DEADBAND) * (MaxAngularRate * 0.8);
            return OIUtil.deadband(-left.getX(), DEADBAND) * (MaxAngularRate * 0.8);
        } else {
            VisionTypes.TargetInfo targetInfo = Subsystems.visionSubsystem.getDefaultLimelight().getTargetInfo();
            // If no target give control bapivotck to human
            if (!targetInfo.hasTarget()) {
//                return OIUtil.deadband(-left.getX(), DEADBAND) * (MaxAngularRate * 0.8);
                return OIUtil.deadband(-left.getX(), DEADBAND) * (MaxAngularRate * 0.8);

            }
//            SmartDashboard.putNumber("TargetXOff", targetInfo.xOffset());
            double horizontalComponent = alignController.calculate(targetInfo.xOffset(), 0);
            double twist = horizontalComponent * Constants.Swerve.kMaxAngularVelocity;
//            DataLogManager.log(("[VISION ALIGNMENT] Running at: " + Timer.getFPGATimestamp() + " | Twist: " + twist));
            return twist;
        }
    }


    private void configureBindings() {
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive
                        .withDeadband(0.02 * MaxSpeed)
//                        .withVelocityX(-right.getY() * MaxSpeed)
//                        .withVelocityY(-right.getX() * MaxSpeed)
                        .withVelocityX(OIUtil.deadband(-right.getY(),0.05) * MaxSpeed)
                        .withVelocityY(OIUtil.deadband(-right.getX(), 0.05) * MaxSpeed)
                        .withRotationalRate(supplySwerveRotate())));


        //
        // Debug/Development
        //

//        xboxController.y().whileTrue(drivetrain.applyRequest(() -> brake));
//        xboxController.b().whileTrue(drivetrain
//                .applyRequest(
//                        () -> point.withModuleDirection(new Rotation2d(-xboxController.getLeftY(), -xboxController.getLeftX()))));

        xboxController.rightBumper().onTrue(Commands.runOnce(Subsystems.pivot::openLoopUp)).onFalse((Commands.runOnce(Subsystems.pivot::holdPosition)));
        xboxController.leftBumper().onTrue(Commands.runOnce(Subsystems.pivot::openLoopDown)).onFalse((Commands.runOnce(Subsystems.pivot::holdPosition)));

        // Test Intake
        if (false) {
            xboxController.leftTrigger()
                    .onTrue(Commands.runOnce(() -> Subsystems.intake.getIntakePivot().pivotOpenLoopDown()))
                    .onFalse(Commands.runOnce(() -> Subsystems.intake.getIntakePivot().holdPosition()));
            xboxController.rightTrigger()
                    .onTrue(Commands.runOnce(() -> Subsystems.intake.getIntakePivot().pivotOpenLoopUp()))
                    .onFalse(Commands.runOnce(() -> Subsystems.intake.getIntakePivot().holdPosition()));
        }
        // Test Climber
        if (true) {
            xboxController.leftTrigger()
                    .onTrue(Commands.runOnce(() -> Subsystems.climber.openLoopUp()))
                    .onFalse(Commands.runOnce(() -> Subsystems.climber.stopOpenLoop()));

            xboxController.rightTrigger()
                    .onTrue(Commands.runOnce(() -> Subsystems.climber.openLoopDown()))
                    .onFalse(Commands.runOnce(() -> Subsystems.climber.stopOpenLoop()));
        }

        // Test Trap
        if (false) {
            xboxController.leftTrigger()
                    .onTrue(Commands.runOnce(() -> Subsystems.trap.getExtender().openLoopDown()))
                    .onFalse(Commands.runOnce(() -> Subsystems.trap.getExtender().stopOpenLoop()));

            xboxController.rightTrigger()
                    .onTrue(Commands.runOnce(() -> Subsystems.trap.getExtender().openLoopUp()))
                    .onFalse(Commands.runOnce(() -> Subsystems.trap.getExtender().stopOpenLoop()));
        }
        xboxController.povUp().onTrue(Commands.runOnce(() -> Subsystems.trap.getExtender().setTrapPosition(TrapExtender.TrapPosition.Up)));
        xboxController.povDown().onTrue(Commands.runOnce(() -> Subsystems.trap.getExtender().setTrapPosition(TrapExtender.TrapPosition.Zero)));



        //
        // Vision Alignment
        //
        runVisionAlignAngle.onTrue(
                        Commands.parallel(
                                Commands.runOnce(() -> this.useVisionAlignment = true),
                                Commands.runOnce(Subsystems.shooter::runShooter),
                                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.ShooterAimVision)))
                .onFalse(
                        Commands.parallel(
                                Commands.runOnce(() -> this.useVisionAlignment = false),
                                Commands.runOnce(Subsystems.shooter::runShooter),
                                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Drive)));


        //
        // Intake Subsystem
        //
        intake.onTrue(Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Pickup))
                .onFalse(Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Drive));

        slowIntake.onTrue(Commands.runOnce(() -> Subsystems.intake.getIntakeSpeed().runIntakeSlow()))
                .onFalse(Commands.runOnce(() -> Subsystems.intake.getIntakeSpeed().stopIntake()));
        eject.onTrue(Commands.runOnce(() -> Subsystems.intake.getIntakeSpeed().runIntakeEject()))
                .onFalse(Commands.runOnce(() -> Subsystems.intake.getIntakeSpeed().stopIntake()));

        xboxController.x().onTrue(Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Pickup));
        xboxController.a().onTrue(Subsystems.poseManager.getPoseCommand(PoseManager.Pose.NotePickedUp));
        xboxController.y().onTrue(Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Drive));

        //xboxController.b().onTrue(Commands.runOnce(() ->
        //        Subsystems.intake.setIntakeState(Intake.IntakeState.TryShootAmp)));


        // Testing
//        xboxController.povUp().onTrue(Commands.runOnce(() -> {
//            double value = SmartDashboard.getNumber("DebugFeederSpeeds", -0.3);
//                    Subsystems.shooter.getFeeder().setOpenLoopSetpoint(value);
//                    Subsystems.intake.getIntakeSpeed().runIntakeDebug(value);[]\
//                    Subsystems.shooter.runFeeder();
//                }))
//                .onFalse(Commands.runOnce(() -> {
//                    Subsystems.shooter.getFeeder().setOpenLoopSetpoint(0.0);
//                    Subsystems.intake.getIntakeSpeed().runIntakeDebug(0.0);
//                }));

        //
        // Shooter
        //
        SmartDashboard.putData("Start Shooter", Commands.runOnce(Subsystems.shooter::runShooter));
        SmartDashboard.putData("Stop Shooter", Commands.runOnce(Subsystems.shooter::stopShooter));

        feed.onTrue(
                Commands.either(
                        Commands.runOnce(Subsystems.shooter::shoot),
                        Commands.runOnce(() -> Subsystems.intake.setIntakeState(Intake.IntakeState.TryShootAmp)),
                        () -> !Subsystems.intake.isNoteDetected())
        ).onFalse(Commands.runOnce(Subsystems.shooter::stopFeeder));

        feed.onTrue(Commands.runOnce(Subsystems.shooter::shoot))
                .onFalse(Commands.runOnce(Subsystems.shooter::stopFeeder));

        xboxController.b().onTrue(Subsystems.poseManager.getPoseCommand(PoseManager.Pose.FeedNoteToShooter));
        ampAim.onTrue(Subsystems.poseManager.getPoseCommand(PoseManager.Pose.PositionForAmp));

        xboxController.start().onTrue(Commands.runOnce(Subsystems.shooter::runShooter));
        xboxController.back().onTrue(Commands.runOnce(Subsystems.shooter::stopShooter));


        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(3, 3), Rotation2d.fromDegrees(0)));
        }
        drivetrain.registerTelemetry(swerveStateTelemetry::telemeterize);

    }

    private void configureDashboardButtons() {
        SmartDashboard.putData("Tare Odometry", new RunWithDisabledInstantCommand(() -> Subsystems.swerveSubsystem.tareEverything()));
        SmartDashboard.putData("Zero Gyro", new ZeroAndSetOffsetCommand(0).ignoringDisable(true));


        SmartDashboard.putData("Update IntakeRotate PID", Subsystems.intake.getIntakePivot().updatePIDFromDashbboardCommand().ignoringDisable(true));
        SmartDashboard.putData("Set IntakeRotater Offset", Subsystems.intake.getIntakePivot().getSetPivotEncoderOffestCmd().ignoringDisable(true));

        SmartDashboard.putData("Pose: Pickup", Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Pickup));
        SmartDashboard.putData("Pose: Handoff", Subsystems.poseManager.getPoseCommand(PoseManager.Pose.NotePickedUp));


        SmartDashboard.putData("Play Lowrida", music.getPlayCommand());
        SmartDashboard.putData("Stop Music", music.getPauseommand());

        SmartDashboard.putData("Set VisionAim State", Subsystems.poseManager.getPoseCommand(PoseManager.Pose.ShooterAimVision));

        // Debug
        SmartDashboard.putNumber("DebugFeederSpeeds", -0.3);
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
