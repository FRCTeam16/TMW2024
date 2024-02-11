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
import frc.robot.commands.VisionAlign;
import frc.robot.commands.ZeroAndSetOffsetCommand;
import frc.robot.commands.auto.RotateToAngle;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Lifecycle;

import java.util.Objects;

public class RobotContainer {
  private static final double MaxSpeed = Constants.Swerve.kMaxSpeedMetersPerSecond;
  private static final double MaxAngularRate = Constants.Swerve.kMaxAngularVelocity;

  private final Joystick left = new Joystick(0);
  private final Joystick right = new Joystick(1);
  private final CommandXboxController xboxController = new CommandXboxController(2); 
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; 
  private final Subsystems subsystems = Subsystems.getInstance();
  private final AutoManager autoManager = new AutoManager();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveStateTelemetry swerveStateTelemtry = new SwerveStateTelemetry(MaxSpeed);

  private final Trigger robotCentric = new Trigger(left::getTrigger);

  //
  // Testing Controls
  //

  private final JoystickButton lockAngle1 = new JoystickButton(left, 8);
  private final JoystickButton lockAngle2 = new JoystickButton(left, 9);

  private final Trigger runVisionAlign = new JoystickButton(right, 3);
  private final JoystickButton runVisionAlignAngle = new JoystickButton(right, 2);

  private final JoystickButton intake = new JoystickButton(right, 1);
  private final JoystickButton slowIntake = new JoystickButton(right, 4);
  private final JoystickButton eject = new JoystickButton(left, 1);

  private final JoystickButton shoot = new JoystickButton(left, 2);
  private final JoystickButton feed = new JoystickButton(left, 3);


  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive
            .withVelocityX(-right.getY() * MaxSpeed)
            .withVelocityY(-right.getX() * MaxSpeed)
            .withRotationalRate(-left.getX() * MaxAngularRate)));
    
    xboxController.y().whileTrue(drivetrain.applyRequest(() -> brake));
    xboxController.b().whileTrue(drivetrain
        .applyRequest(
            () -> point.withModuleDirection(new Rotation2d(-xboxController.getLeftY(), -xboxController.getLeftX()))));



    // reset the field-centric heading on left bumper press
    // FIXME this does not work
    robotCentric.onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative));

    // Intake Subsystem
    intake.onTrue(new InstantCommand(()    -> Subsystems.intake.IntakePart())).onFalse(new InstantCommand(() -> Subsystems.intake.OpenLoopStop()));
    slowIntake.onTrue(new InstantCommand(()    -> Subsystems.intake.SlowIntake())).onFalse(new InstantCommand(() -> Subsystems.intake.OpenLoopStop()));
    eject.onTrue(new InstantCommand(()     -> Subsystems.intake.Eject())).onFalse( new InstantCommand(()   -> Subsystems.intake.OpenLoopStop()));


    // Shooter
    shoot.onTrue(Commands.runOnce(Subsystems.shooter::runShooterOpenLoop))
            .onFalse(Commands.runOnce(Subsystems.shooter::stopShooter));
    feed.onTrue(Commands.runOnce(Subsystems.shooter::runFeeder))
            .onFalse(Commands.runOnce(Subsystems.shooter::stopFeeder));


    // Debug
    lockAngle1.onTrue(new RotateToAngle(-60));
    lockAngle2.onTrue(new RotateToAngle(0));

    runVisionAlign.whileTrue(new VisionAlign());
    runVisionAlignAngle.whileTrue(new VisionAlign().withRobotAngle(90.0));


    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(swerveStateTelemtry::telemeterize);


    //TODO: add if to prevent comp bot subsystem calls on practice bot

    intake.onTrue(new InstantCommand(()    -> subsystems.intake.IntakePart())).onFalse(new InstantCommand(() -> subsystems.intake.OpenLoopStop()));
    slowIntake.onTrue(new InstantCommand(()    -> subsystems.intake.SlowIntake())).onFalse(new InstantCommand(() -> subsystems.intake.OpenLoopStop()));
    eject.onTrue(new InstantCommand(()     -> subsystems.intake.Eject())).onFalse( new InstantCommand(()   -> subsystems.intake.OpenLoopStop()));
}

  public RobotContainer() {
    configureBindings();
    configureDashboardButtons();
  }

  private void configureDashboardButtons() {
    SmartDashboard.putData("Tare Odometry", new RunWithDisabledInstantCommand( () -> Subsystems.swerveSubsystem.tareEverything()));
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
