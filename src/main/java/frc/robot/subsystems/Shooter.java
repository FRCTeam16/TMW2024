package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase implements Lifecycle{
    private TalonFX beltDrive_1 = new TalonFX(25000); // give real number
    private TalonFX beltDrive_2 = new TalonFX(25001); // give real number

    private TalonFX noteFeederDrive = new TalonFX(25002); // give real number
    private TalonFX aimDrive = new TalonFX(25003); // give real numuber
    
}
