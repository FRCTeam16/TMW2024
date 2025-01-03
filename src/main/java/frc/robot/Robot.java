// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.config.RobotConfiguration;
import frc.robot.config.BuildConstants;
import frc.robot.subsystems.PowerTelemetry;
import frc.robot.subsystems.util.BSLogger;
import frc.robot.subsystems.vision.VisionTypes;

public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private RobotContainer robotContainer;

  private PowerTelemetry powerTelemetry = new PowerTelemetry();

  @Override
  public void robotInit() {
    DataLogManager.start();
    RobotConfiguration.initialize();
    robotContainer = new RobotContainer();

    // Log Startup Message
    //noinspection ConstantValue
    BSLogger.log("Robot",
            String.format("""
                            *********************************
                            *** Deployed Code Information ***
                            Project Name: %s
                            Branch: %s
                            Dirty: %s
                            Build Date: %s
                            Git SHA: %s
                            Git Date: %s
                            *********************************
                            """, BuildConstants.MAVEN_NAME, BuildConstants.GIT_BRANCH,
                    BuildConstants.DIRTY == 1, BuildConstants.BUILD_DATE,
                    BuildConstants.GIT_SHA, BuildConstants.GIT_DATE));

    Subsystems.swerveSubsystem.getPigeon2().setYaw(0);

//    for (int i=0;i<4;i++) {
//      Subsystems.swerveSubsystem.getModule(i).getDriveMotor().setNeutralMode(NeutralModeValue.Brake);
//    }


    // Forward LimeLight ports so they are available over USB
//    for (int port = 5800; port <= 5807; port++) {
//        PortForwarder.add(port, "limelight.local", port);
//    }


    addPeriodic(Subsystems.ledSubsystem::Report, 0.1);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    robotContainer.robotPeriodic();

    VisionTypes.TargetInfo info = Subsystems.visionSubsystem.getDefaultLimelight().getTargetInfo();
    SmartDashboard.putNumber("VisionTestTarget/Distance", info.calculateDistance());

    powerTelemetry.periodic();
  }

  @Override
  public void disabledInit() {
    Subsystems.intake.getIntakePivot().resetEncoderOnce();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    BSLogger.log("Robot", "autoInit:: Started at:" + Timer.getFPGATimestamp());
    autonomousCommand = robotContainer.getAutonomousCommand();
    BSLogger.log("Robot", "autoInit:: got robotCommand: " + Timer.getFPGATimestamp());
    robotContainer.autoInit();
    BSLogger.log("Robot", "autoInit:: robot container autoInit finished: " + Timer.getFPGATimestamp());

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
      BSLogger.log("Robot", "autoInit:: scheduled command at: " + Timer.getFPGATimestamp());
    }
    BSLogger.log("Robot", "autoInit:: finished at: " + Timer.getFPGATimestamp());
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    for (int i=0;i<4;i++) {
      Subsystems.swerveSubsystem.getModule(i).getDriveMotor().setNeutralMode(NeutralModeValue.Coast);
    }
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    robotContainer.teleopInit();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
