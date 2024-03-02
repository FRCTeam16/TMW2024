package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.RunWithDisabledInstantCommand;
import frc.robot.commands.ZeroAndSetOffsetCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.RotationController;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pose.PoseManager;
import frc.robot.subsystems.trap.TrapExtender;
import frc.robot.subsystems.vision.VisionTypes;

import java.util.Objects;

import static edu.wpi.first.units.Units.Radians;

public class RobotContainer {
    private static final double MaxSpeed = Constants.Swerve.kMaxSpeedMetersPerSecond;
    private static final double MaxAngularRate = Constants.Swerve.kMaxAngularVelocity;

    private final Joystick left = new Joystick(0);
    private final Joystick right = new Joystick(1);
    private final CommandXboxController xboxController = new CommandXboxController(2);
    private final XboxController debugJoystick = new XboxController(3);

    private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

    @SuppressWarnings("unused")
    private final Subsystems subsystems = Subsystems.getInstance();

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.01)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.01)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveStateTelemetry swerveStateTelemetry = new SwerveStateTelemetry(MaxSpeed);

    //
    // Left Joystick
    //
    private final JoystickButton intake = new JoystickButton(left, 1);
    private final JoystickButton eject = new JoystickButton(left, 2);
    private final JoystickButton feedIntake = new JoystickButton(left, 4);  // debug feed intake speeds


    //
    // Right Joystick
    //
    private final Trigger feed = new Trigger(right::getTrigger);
    private final JoystickButton runVisionAlignAngle = new JoystickButton(right, 2);
    //private final JoystickButton feedNote = new JoystickButton(right, 3);
    private final JoystickButton ampAim = new JoystickButton(right, 4);
    private final Trigger debugLeftTrigger = new Trigger(() -> debugJoystick.getLeftTriggerAxis() > 0.1);
    private final Trigger debugRightTrigger = new Trigger(() -> debugJoystick.getRightTriggerAxis() > 0.1);

    //
    // Controller
    //
    private final Trigger startShooter = xboxController.start();
    private final Trigger stopShooter = xboxController.back();
    private final Trigger rightBumper = xboxController.rightBumper();
    private final Trigger leftBumper = xboxController.leftBumper();

    private final Trigger leftTrigger = xboxController.leftTrigger();
    private final Trigger rightTrigger = xboxController.rightTrigger();

    private final Trigger feedNoteToShooter = xboxController.b();
    private final Trigger startClimb = xboxController.x();


    //
    // Miscellaneous
    //
    private final RotationController alignController = new RotationController(0.02, 0, 0);
    Trigger povUp = xboxController.povUp();
    Trigger povDown = xboxController.povDown();
    MusicController music = new MusicController();
    private boolean useVisionAlignment = false;

    public RobotContainer() {
        configureBindings();
        configureDashboardButtons();
        alignController.setTolerance(0.05);
        SmartDashboard.putData("AlignPID", alignController);

    }

    /**
     * This method is used to supply the swerve rotate value to the swerve drive
     *
     * @return the swerve rotate value
     */
    private Measure<Angle> supplySwerveRotate() {
        final double DEADBAND = 0.05;
        final double twist;
        if (!useVisionAlignment) {
            twist = OIUtil.deadband(-left.getX(), DEADBAND) * (MaxAngularRate * 0.8);
        } else {
            VisionTypes.TargetInfo targetInfo = Subsystems.visionSubsystem.getDefaultLimelight().getTargetInfo();
            // If no target give control back to human inputs
            if (!targetInfo.hasTarget()) {
                twist = OIUtil.deadband(-left.getX(), DEADBAND) * (MaxAngularRate * 0.8);
            } else {
                double horizontalComponent = alignController.calculate(targetInfo.xOffset(), 0);
                twist = horizontalComponent * Constants.Swerve.kMaxAngularVelocity;
            }
        }
        return Radians.of(twist);
    }


    private void configureBindings() {
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive
                        .withDeadband(0.02 * MaxSpeed)
                        .withVelocityX(OIUtil.deadband(-right.getY(), 0.05) * MaxSpeed)
                        .withVelocityY(OIUtil.deadband(-right.getX(), 0.05) * MaxSpeed)
                        .withRotationalRate(supplySwerveRotate().in(Radians))));


        //
        // Debug/Development
        //
        rightBumper.onTrue(Commands.runOnce(Subsystems.pivot::openLoopUp)).onFalse((Commands.runOnce(Subsystems.pivot::holdPosition)));
        leftBumper.onTrue(Commands.runOnce(Subsystems.pivot::openLoopDown)).onFalse((Commands.runOnce(Subsystems.pivot::holdPosition)));

        xboxController.povLeft().onTrue(Commands.runOnce(() -> Subsystems.climber.setClimberPosition(Climber.ClimberPosition.DOWN)));
        xboxController.povRight().onTrue(Commands.runOnce(() -> Subsystems.climber.setClimberPosition(Climber.ClimberPosition.UP)));

        // Test Intake
        if (false) {
            leftTrigger
                    .onTrue(Commands.runOnce(() -> Subsystems.intake.getIntakePivot().pivotOpenLoopDown()))
                    .onFalse(Commands.runOnce(() -> Subsystems.intake.getIntakePivot().holdPosition()));
            rightTrigger
                    .onTrue(Commands.runOnce(() -> Subsystems.intake.getIntakePivot().pivotOpenLoopUp()))
                    .onFalse(Commands.runOnce(() -> Subsystems.intake.getIntakePivot().holdPosition()));
        }
        // Test Climber
        if (true) {
            leftTrigger
                    .onTrue(Commands.runOnce(() -> Subsystems.climber.openLoopUp()))
                    .onFalse(Commands.runOnce(() -> Subsystems.climber.stopOpenLoop()));

            rightTrigger
                    .onTrue(Commands.runOnce(() -> Subsystems.climber.openLoopDown()))
                    .onFalse(Commands.runOnce(() -> Subsystems.climber.stopOpenLoop()));
        }

        // Test Trap
        if (true) {
            debugLeftTrigger
                    .onTrue(Commands.runOnce(() -> Subsystems.trap.getExtender().openLoopDown()))
                    .onFalse(Commands.runOnce(() -> Subsystems.trap.getExtender().stopOpenLoop()));

            debugRightTrigger
                    .onTrue(Commands.runOnce(() -> Subsystems.trap.getExtender().openLoopUp()))
                    .onFalse(Commands.runOnce(() -> Subsystems.trap.getExtender().stopOpenLoop()));
        }

        povUp.onTrue(Commands.runOnce(() -> Subsystems.trap.getExtender().setTrapPosition(TrapExtender.TrapPosition.Up)));
        povDown.onTrue(Commands.runOnce(() -> Subsystems.trap.getExtender().setTrapPosition(TrapExtender.TrapPosition.Zero)));


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
        feedIntake.onTrue(Commands.runOnce(() -> Subsystems.intake.getIntakeSpeed().runIntakeFeedShooter()))
                .onFalse(Commands.runOnce(() -> Subsystems.intake.getIntakeSpeed().stopIntake()));
        eject.onTrue(Commands.runOnce(() -> Subsystems.intake.getIntakeSpeed().runIntakeEject()))
                .onFalse(Commands.runOnce(() -> Subsystems.intake.getIntakeSpeed().stopIntake()));


        //
        // Climb subsystem
        //
        startClimb.onTrue(Subsystems.poseManager.getPoseCommand(PoseManager.Pose.StartClimb));


        //
        // Shooter
        //
        feed.onTrue(
                Commands.either(
                        Commands.runOnce(Subsystems.shooter::shoot),
                        Commands.runOnce(() -> Subsystems.intake.setIntakeState(Intake.IntakeState.TryShootAmp)),
                        () -> !Subsystems.intake.isNoteDetected())
        ).onFalse(Commands.runOnce(Subsystems.shooter::stopFeeder));

        feedNoteToShooter.onTrue(Subsystems.poseManager.getPoseCommand(PoseManager.Pose.FeedNoteToShooter));
        ampAim.onTrue(Subsystems.poseManager.getPoseCommand(PoseManager.Pose.PositionForAmp));

        startShooter.onTrue(Commands.runOnce(Subsystems.shooter::runShooter));
        stopShooter.onTrue(Commands.runOnce(Subsystems.shooter::stopShooter));


        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(3, 3), Rotation2d.fromDegrees(0)));
        }
        drivetrain.registerTelemetry(swerveStateTelemetry::telemeterize);

    }

    private void configureDashboardButtons() {
        SmartDashboard.putData("Start Shooter", Commands.runOnce(Subsystems.shooter::runShooter));
        SmartDashboard.putData("Stop Shooter", Commands.runOnce(Subsystems.shooter::stopShooter));

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
        return Subsystems.autoManager.getSelectedAutoStrategy();
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
