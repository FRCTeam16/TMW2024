package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems;
import frc.robot.subsystems.Lifecycle;


public class Intake extends SubsystemBase implements Lifecycle{
    
    private TalonFX intakeDrive = new TalonFX(0); // google tells me the Krakens are on TalonFX for code
    private TalonFX pivotDrive = new TalonFX(1); // this is geared down 48:1

    private TalonFXConfiguration defaultConfig = new TalonFXConfiguration();
 
    private TalonFXConfigurator intakeDrive_configurer = intakeDrive.getConfigurator();
    private TalonFXConfigurator pivotDrive_configurer = intakeDrive.getConfigurator();

    private final DutyCycleOut intakeDrive_Request = new DutyCycleOut(0.0); // CTRE pheonix 6 API
    private final DutyCycleOut pivotDrive_Request = new DutyCycleOut(0.0);

    private double INTAKE_MAX_SPEED = 1;
    private double INTAKE_SLOW_SPEED = 0.25;

    private boolean OpenLoop = false;
        


    public Intake(){
        intakeDrive_configurer.apply(defaultConfig);
        pivotDrive_configurer.apply(defaultConfig);
        
        
    }

    public void OpenLoopDriveIntake(){
        OpenLoop = true;
    }

    public void OpenLoopStop(){
        OpenLoop = false;
    }

    @Override
    public void periodic(){
        if(OpenLoop){
            intakeDrive.setControl(intakeDrive_Request.withOutput(INTAKE_SLOW_SPEED)); // go brr code?
            
        } else {
            //TODO: UUHHH DO THIS
        }     
    }

    




}
