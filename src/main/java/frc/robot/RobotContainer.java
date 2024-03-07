package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CenterNoteIntakeCommand;
import frc.robot.commands.RunWithDisabledInstantCommand;
import frc.robot.commands.TeleopShoot;
import frc.robot.commands.ZeroAndSetOffsetCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.RotationController;
import frc.robot.subsystems.pose.ClimbManager;
import frc.robot.subsystems.pose.PoseManager;
import frc.robot.subsystems.trap.Trap;
import frc.robot.subsystems.trap.TrapPivot;
import frc.robot.subsystems.util.BSLogger;
import frc.robot.subsystems.util.GameInfo;
import frc.robot.subsystems.vision.VisionTypes;

import java.util.Objects;

import static edu.wpi.first.units.Units.RadiansPerSecond;

public class RobotContainer {
    private static final double MaxSpeed = Constants.Swerve.kMaxSpeedMetersPerSecond;
    private static final double MaxAngularRate = Constants.Swerve.kMaxAngularVelocity;
    //
    // Vision integration
    //
    private static boolean useVisionAlignment = false; // twist alignment for auto-aiming
    private final Joystick left = new Joystick(0);
    private final Joystick right = new Joystick(1);
    private final CommandXboxController xboxController = new CommandXboxController(2);
//    private final XboxController debugJoystick = new XboxController(3);
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
    private final JoystickButton bigShot = new JoystickButton(left, 2);

    private final JoystickButton feedIntake = new JoystickButton(left, 4);  // debug feed intake speeds
    private final JoystickButton unsafeRaiseClimber = new JoystickButton(left, 14);
    private final JoystickButton unsafeLowerClimber = new JoystickButton(left, 15);
    private final JoystickButton unsafeLowerTrap = new JoystickButton(left, 8);
    private final JoystickButton unsafeRaiseTrap = new JoystickButton(left, 9);
    //
    // Right Joystick
    //
    private final Trigger feed = new Trigger(right::getTrigger);
    private final JoystickButton runVisionAlignAngle = new JoystickButton(right, 2);
    private final JoystickButton robotCentric = new JoystickButton(right, 5);
    private final JoystickButton shootOverSmiley = new JoystickButton(right, 4);

    private final JoystickButton bumpClimberDown = new JoystickButton(right, 6);
    private final JoystickButton bumpClimberUp = new JoystickButton(right, 7);
    private final Trigger subShot = new POVButton(right, 0);
            
    private final JoystickButton tryClearNote = new JoystickButton(right, 16);
    //
    // Debug Controller
    //
    /*
    private final Trigger debugLeftTrigger = new Trigger(() -> debugJoystick.getLeftTriggerAxis() > 0.1);
    private final Trigger debugRightTrigger = new Trigger(() -> debugJoystick.getRightTriggerAxis() > 0.1);
    private final Trigger debugYButton = new JoystickButton(debugJoystick, 4);
    private final Trigger debugAButton = new JoystickButton(debugJoystick, 1);
    private final Trigger debugBButton = new JoystickButton(debugJoystick, 2);
    private final Trigger debugXButton = new JoystickButton(debugJoystick, 3);
    */

    //
    // Controller
    //
    private final Trigger startShooter = xboxController.start();
    private final Trigger stopShooter = xboxController.back();
    private final Trigger rightBumper = xboxController.rightBumper();
    private final Trigger leftBumper = xboxController.leftBumper();
    private final Trigger leftTrigger = xboxController.leftTrigger();
    private final Trigger rightTrigger = xboxController.rightTrigger();
    private final Trigger ampAim = xboxController.x();
    private final Trigger feedNoteToShooter = xboxController.y();
    private final Trigger startClimb = xboxController.povUp();
    private final Trigger climbPull = xboxController.povDown();

    private final Trigger eject = xboxController.b();


    //
    // Miscellaneous
    //
    private final RotationController alignController = new RotationController(0.02, 0, 0);
    //    Trigger povUp = xboxController.povUp();
//    Trigger povDown = xboxController.povDown();
    MusicController music = new MusicController();

//    private final VisionAlignmentHelper trapAlignHelper = new VisionAlignmentHelper();
//    private boolean useVisionTrapAligment = false; // horizontal input for trap alignment

    boolean isRedAlliance;

    public RobotContainer() {
        configureBindings();
        configureDashboardButtons();
        alignController.setTolerance(0.05);
        SmartDashboard.putData("AlignPID", alignController);
        SmartDashboard.setDefaultNumber("AlignPIDFactor", 200);

        updateRedAllianceInfo();
    }
    

    public void updateRedAllianceInfo() {
        isRedAlliance = GameInfo.isRedAlliance();
        BSLogger.log("RobotContainer", "updateRedAllianceInfo = " + isRedAlliance);
    }

    public static boolean isUseVisionAlignment() {
        return useVisionAlignment;
    }

    public double supplySwerveX() {
        double base = -right.getY();
        return (isRedAlliance) ? -1 * base : base;
    }

    public double supplySwerveY() {
        double base = -right.getX();
        return isRedAlliance ? -1 * base : base;
    }

    /**
     * This method is used to supply the swerve rotate value to the swerve drive
     *
     * @return the swerve rotate value
     */
    private Measure<Velocity<Angle>> supplySwerveRotate() {
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
                double distance = targetInfo.calculateDistance();
                double factor = distance / SmartDashboard.getNumber("AlignPIDFactor", 200);
                double horizontalComponent = factor * alignController.calculate(targetInfo.xOffset(), 0);
                twist = MathUtil.clamp(horizontalComponent * Constants.Swerve.kMaxAngularVelocity, -Math.PI, Math.PI);
            }
        }
        return RadiansPerSecond.of(twist);
    }

    /**
     * This method is used to supply the swerve horizontal value to the swerve drive
     * <p>
     * Will be wired in if/when second limelight is added.  Will also need trap-specific AprilTag pipeline to use
     *
     * @return the swerve horizontal value
     */
//    private Measure<Velocity<Distance>> supplySwerveHorizontal() {
//        if (!useVisionTrapAligment) {
//            return MetersPerSecond.of(OIUtil.deadband(-right.getY(), 0.05) * MaxSpeed);
//        } else {
//            VisionTypes.TargetInfo targetInfo = Subsystems.visionSubsystem.getDefaultLimelight().getTargetInfo();
//            if (!targetInfo.hasTarget()) {
//                return MetersPerSecond.of(OIUtil.deadband(-right.getY(), 0.05) * MaxSpeed);
//            } else {
//                double horizontalComponent = alignController.calculate(targetInfo.yOffset(), 0);
//                return MetersPerSecond.of(horizontalComponent * MaxSpeed);
//            }
//        }
//    }
    private void configureBindings() {
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive
                        .withDeadband(0.02 * MaxSpeed)
                        .withVelocityX(OIUtil.deadband(supplySwerveX(), 0.05) * MaxSpeed)
                        .withVelocityY(OIUtil.deadband(supplySwerveY(), 0.05) * MaxSpeed)
                        .withRotationalRate(supplySwerveRotate().in(RadiansPerSecond))));

        robotCentric.whileTrue(
                drivetrain.applyRequest(() -> robotCentricDrive
                        .withDeadband(0.02 * MaxSpeed)
                        .withVelocityX(OIUtil.deadband(supplySwerveY(), 0.05) * MaxSpeed)
                        .withVelocityY(OIUtil.deadband(supplySwerveY(), 0.05) * MaxSpeed)
                        .withRotationalRate(supplySwerveRotate().in(RadiansPerSecond))));


        //
        // Debug/Development
        //
        rightBumper.onTrue(Commands.runOnce(Subsystems.pivot::openLoopUp)).onFalse((Commands.runOnce(Subsystems.pivot::holdPosition)));
        leftBumper.onTrue(Commands.runOnce(Subsystems.pivot::openLoopDown)).onFalse((Commands.runOnce(Subsystems.pivot::holdPosition)));


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
        if (false) {
            leftTrigger
                    .onTrue(Commands.runOnce(() -> Subsystems.climber.openLoopUp()))
                    .onFalse(Commands.runOnce(() -> Subsystems.climber.stopOpenLoop()));

            rightTrigger
                    .onTrue(Commands.runOnce(() -> Subsystems.climber.openLoopDown()))
                    .onFalse(Commands.runOnce(() -> Subsystems.climber.stopOpenLoop()));
        }

        // Test Trap
        if (false) {
            /*
            debugLeftTrigger
                    .onTrue(Commands.runOnce(() -> Subsystems.trap.getExtender().openLoopDown()))
                    .onFalse(Commands.runOnce(() -> Subsystems.trap.getExtender().stopOpenLoop()));

            debugRightTrigger
                    .onTrue(Commands.runOnce(() -> Subsystems.trap.getExtender().openLoopUp()))
                    .onFalse(Commands.runOnce(() -> Subsystems.trap.getExtender().stopOpenLoop()));

//            debugYButton.onTrue(Commands.runOnce(() -> Subsystems.trap.getPivot().openLoopUp()))
//                    .onFalse(Commands.runOnce(() -> Subsystems.trap.getPivot().stopOpenLoop()));
//            debugAButton.onTrue(Commands.runOnce(() -> Subsystems.trap.getPivot().openLoopDown()))
//                    .onFalse(Commands.runOnce(() -> Subsystems.trap.getPivot().stopOpenLoop()));

            debugYButton.onTrue(Commands.runOnce(() -> Subsystems.trap.getPivot().setTrapPosition(TrapPivot.TrapPivotPosition.Top)));
            debugAButton.onTrue(Commands.runOnce(() -> Subsystems.trap.getPivot().setTrapPosition(TrapPivot.TrapPivotPosition.Drive)));


            debugBButton.onTrue(Commands.runOnce(() -> Subsystems.trap.setFingerPosition(Trap.FingerPositions.Closed)));
            debugXButton.onTrue(Commands.runOnce(() -> Subsystems.trap.setFingerPosition(Trap.FingerPositions.Open)));
*/
        }

        // Test Trap
        if (true) {
            leftTrigger
                    .onTrue(Commands.runOnce(() -> Subsystems.trap.getPivot().openLoopUp()))
                    .onFalse(Commands.runOnce(() -> Subsystems.trap.getPivot().stopOpenLoop()));

            rightTrigger
                    .onTrue(Commands.runOnce(() -> Subsystems.trap.getPivot().openLoopDown()))
                    .onFalse(Commands.runOnce(() -> Subsystems.trap.getPivot().stopOpenLoop()));
        }


        //
        // Vision Alignment
        //
        runVisionAlignAngle.whileTrue(
                        Commands.parallel(
                                Commands.runOnce(() -> useVisionAlignment = true),
                                Commands.runOnce(Subsystems.shooter::runShooter),
                                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.ShooterAimVision)))
                .onFalse(
                        Commands.parallel(
                                Commands.runOnce(() -> useVisionAlignment = false),
                                Commands.runOnce(Subsystems.shooter::runShooter),   // reset to default
                                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Drive)));

        //
        tryClearNote.whileTrue(Subsystems.poseManager.getPoseCommand(PoseManager.Pose.TryClearNote))
                .onFalse(Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Drive));


        //
        // Intake Subsystem
        //
        intake.whileTrue(Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Pickup))
                .onFalse(Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Drive));
        feedIntake.onTrue(Commands.runOnce(() -> Subsystems.intake.getIntakeSpeed().runIntakeFeedShooter()))
                .onFalse(Commands.runOnce(() -> Subsystems.intake.getIntakeSpeed().stopIntake()));
        eject.onTrue(Commands.runOnce(() -> Subsystems.intake.getIntakeSpeed().runIntakeEject()))
                .onFalse(Commands.runOnce(() -> Subsystems.intake.getIntakeSpeed().stopIntake()));


        //
        // Climb subsystem
        //
//        startClimb.onTrue(Subsystems.poseManager.getPoseCommand(PoseManager.Pose.StartClimb));
        startClimb.onTrue(
                Subsystems.poseManager.getClimbManager().getPoseCommand(ClimbManager.ClimbPose.StartClimb));
        climbPull.onTrue(
                Subsystems.poseManager.getClimbManager().getPoseCommand(ClimbManager.ClimbPose.PullUp));

        unsafeLowerClimber.onTrue(Commands.runOnce(() -> Subsystems.climber.unsafeOpenLoopUp()))
                .onFalse(Commands.runOnce(() -> Subsystems.climber.stopOpenLoop()));
        unsafeRaiseClimber.onTrue(Commands.runOnce(() -> Subsystems.climber.unsafeOpenLoopDown()))
                .onFalse(Commands.runOnce(() -> Subsystems.climber.stopOpenLoop()));

//        climberDown.onTrue(Commands.runOnce(() -> Subsystems.climber.setClimberPosition(Climber.ClimberPosition.DOWN)));
//        climberUp.onTrue(Commands.runOnce(() -> Subsystems.climber.setClimberPosition(Climber.ClimberPosition.UP)));
        bumpClimberUp.onTrue(Commands.runOnce(() -> Subsystems.climber.bumpSetpoint(0.5))); //TODO: change to a tested number
        bumpClimberDown.onTrue(Commands.runOnce(() -> Subsystems.climber.bumpSetpoint(-0.5)));


        //
        // Trap Subsystem
        //
        unsafeLowerTrap.onTrue(Commands.runOnce(() -> Subsystems.trap.getExtender().unsafeOpenLoopDown()))
                .onFalse(Commands.runOnce(() -> Subsystems.trap.getExtender().stopOpenLoop()));
        unsafeRaiseTrap.onTrue(Commands.runOnce(() -> Subsystems.trap.getExtender().unsafeOpenLoopUp()))
                .onFalse(Commands.runOnce(() -> Subsystems.trap.getExtender().stopOpenLoop()));


        //
        // Shooter
        //
        feed.onTrue(new TeleopShoot())
                .onFalse(Commands.runOnce(Subsystems.shooter::stopFeeder));




        feedNoteToShooter.onTrue(Subsystems.poseManager.getPoseCommand(PoseManager.Pose.FeedNoteToShooter));

        ampAim.onTrue(Subsystems.poseManager.getPoseCommand(PoseManager.Pose.PositionForAmp));

        startShooter.onTrue(Commands.runOnce(Subsystems.shooter::runShooter));
        stopShooter.onTrue(Commands.runOnce(Subsystems.shooter::stopShooter));

        bigShot.onTrue(
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.FireBigShot)
        ).onFalse(Commands.runOnce(Subsystems.shooter::stopFeeder));

        shootOverSmiley.onTrue(
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.FireShootOverSmiley)
        ).onFalse(Commands.runOnce(Subsystems.shooter::stopFeeder));

        subShot.onTrue(
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.SubShot)
        ).onFalse(Commands.runOnce(Subsystems.shooter::stopFeeder));


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
        SmartDashboard.putData("Fix Intake Pivot Offset", Subsystems.intake.getIntakePivot().fixMotorPositionCmd().ignoringDisable(true));

        SmartDashboard.putData("Pose: Pickup", Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Pickup));
        SmartDashboard.putData("Pose: Handoff", Subsystems.poseManager.getPoseCommand(PoseManager.Pose.NotePickedUp));

        SmartDashboard.putData("Play Lowrida", music.getPlayCommand());
        SmartDashboard.putData("Stop Music", music.getPauseommand());

        SmartDashboard.putData("Set VisionAim State", Subsystems.poseManager.getPoseCommand(PoseManager.Pose.ShooterAimVision));

        // Debug
        SmartDashboard.setDefaultNumber("DeflectDelay", 0.75);

        SmartDashboard.putData("Center Note in intake", new CenterNoteIntakeCommand());

        SmartDashboard.putData("TestCmd",
                Commands.sequence(
                    Subsystems.poseManager.getPoseCommand(PoseManager.Pose.PositionForAmp),
                    new TeleopShoot().ignoringDisable(true)).ignoringDisable(true)
        );
    }

    public Command getAutonomousCommand() {
        return Subsystems.autoManager.getSelectedAutoStrategy();
    }

    public void teleopInit() {
        updateRedAllianceInfo();
        useVisionAlignment = false;
        Subsystems.lifecycleSubsystems.stream().filter(Objects::nonNull).forEach(Lifecycle::teleopInit);
    }

    public void autoInit() {
        useVisionAlignment = false;
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

        SmartDashboard.putBoolean("Debug/UseVisionAlignment", useVisionAlignment);
    }

}
