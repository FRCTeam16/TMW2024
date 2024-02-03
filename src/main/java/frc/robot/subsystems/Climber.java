package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems;
import frc.robot.subsystems.Lifecycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase implements Lifecycle{
   //motor configuration
    private TalonFX climberDrive = new TalonFX(11);//set to 125:1

  //make climber go up and down
  private static double moveUp = .1; //arbitrary number
  private static double moveDown = .1; //arbitrary number

  private double setpoint = 0.0;
  public enum ClimberPosition{
    Stow(0),
    up(100);
public final double setpoint;

private ClimberPosition(double setpoint) {
  this.setpoint = setpoint;
  }

  //climbing action
  public Climb(){
this.ClimberPosition = ClimberPosition.up;
this.intake = intake.down;//sample number
//same with the enum positions of both the intake and shooter
  }
}
