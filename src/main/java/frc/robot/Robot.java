// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    DataLogManager.start();
    m_robotContainer = new RobotContainer();
    Subsystems.swerveSubsystem.getPigeon2().setYaw(0);

    /*
     * addPeriodic(Subsystems.ledSubsystem::Report, 0.1);
     */
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    SmartDashboard.putNumber("Drive/robotAngle", Subsystems.swerveSubsystem.getPigeon2().getAngle());
    SmartDashboard.putNumber("Drive/RawYaw", Subsystems.swerveSubsystem.getPigeon2().getYaw().getValueAsDouble());
    SmartDashboard.putNumber("Drive/swerveYaw", Subsystems.swerveSubsystem.getYaw());
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
