// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.config.BuildConstants;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    DataLogManager.start();
    m_robotContainer = new RobotContainer();

    // Log Startup Message
    DataLogManager.log(
            String.format("""
                            *********************************
                            *** Deployed Code Informatiom ***
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


    // Forward LimeLight ports so they are available over USB
    for (int port = 5800; port <= 5807; port++) {
        PortForwarder.add(port, "limelight.local", port);
    }

    /*
     * addPeriodic(Subsystems.ledSubsystem::Report, 0.1);
     */
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.robotPeriodic();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    m_robotContainer.autoInit();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.teleopInit();
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
