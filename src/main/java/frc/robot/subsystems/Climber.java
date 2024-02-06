package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase implements Lifecycle {
  // motor configuration
  private TalonFX climberDrive = new TalonFX(11);// set to 125:1

  // make climber go up and down
  private static double moveUp = .1; // arbitrary number
  private static double moveDown = .1; // arbitrary number

  private double setpoint = 0.0;

  public enum ClimberPosition {
    Stow(0),
    up(100);

    public final double setpoint;

    private ClimberPosition(double setpoint) {
      this.setpoint = setpoint;
    }
  }

  private ClimberPosition currentPosition = ClimberPosition.Stow;

  public Climber() {
    // TODO: enforce strict limits during development to prevent breaking
  }

  // climbing action
  public void climb() {
    this.currentPosition = ClimberPosition.up;
    // TODO: Need to synchronize actions with other subsystems
  }
}