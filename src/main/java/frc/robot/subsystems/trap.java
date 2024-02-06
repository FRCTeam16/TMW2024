package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class trap extends SubsystemBase implements Lifecycle{
  
  private TalonFX rotationMotor = new TalonFX(129); //TODO: give actuall number
  private Servo finger1 = new Servo(1); // I dont know what 1 means but yea
  private Servo finger2 = new Servo(2);
  private Servo pivoter = new Servo(3);

  private boolean OpenLoop = false;

  public void runOpenLoop(){
    OpenLoop = true;

  }



  @Override
  public static void periodic(){
    // do stuff logic
  }





}




