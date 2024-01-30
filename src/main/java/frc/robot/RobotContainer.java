// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AutoManager;
import frc.robot.commands.RunWithDisabledInstantCommand;
import frc.robot.commands.VisionAlign;
import frc.robot.commands.ZeroAndSetOffsetCommand;
import frc.robot.commands.auto.RotateToAngle;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  private static final double MaxSpeed = Constants.Swerve.kMaxSpeedMetersPerSecond;
  private static final double MaxAngularRate = Constants.Swerve.kMaxAngularVelocity;

  /* Setting up bindings for necessary control of the swerve drive platform */
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
  private final Trigger shooterPrototypeOpenLoop = xboxController.x();
  private final Trigger shooterPrototypeClosedLoop = xboxController.a();

  private final JoystickButton lockAngle1 = new JoystickButton(left, 8);
  private final JoystickButton lockAngle2 = new JoystickButton(left, 9);

  private final Trigger runVisionAlign = new Trigger(right::getTrigger);


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

    // shooterPrototypeOpenLoop.onTrue(new InstantCommand(Subsystems.shooterPrototype::toggleOpenLoop));
    // shooterPrototypeClosedLoop.onTrue(new InstantCommand(Subsystems.shooterPrototype::toggleClosedLoop));


    // reset the field-centric heading on left bumper press
    robotCentric.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    lockAngle1.onTrue(new RotateToAngle(-60));
    lockAngle2.onTrue(new RotateToAngle(0));

    runVisionAlign.whileTrue(new VisionAlign());


    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(swerveStateTelemtry::telemeterize);
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
    Subsystems.lifecycleSubsystems.stream().filter(s -> s != null).forEach((s) -> s.teleopInit());
  }

  public void autoInit() {
    Subsystems.lifecycleSubsystems.stream().filter(s -> s != null).forEach((s) -> s.autoInit());
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
