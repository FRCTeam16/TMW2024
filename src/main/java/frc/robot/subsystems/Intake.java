package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems;
import frc.robot.subsystems.Lifecycle;


public class Intake extends SubsystemBase implements Lifecycle{
    
    private TalonFX intakeDrive = new TalonFX(9); // google tells me the Krakens are on TalonFX for code
    private TalonFX pivotDrive = new TalonFX(10); // this is geared down 48:1

    private TalonFXConfiguration defaultConfig = new TalonFXConfiguration();
 
    private TalonFXConfigurator intakeDrive_configurer = intakeDrive.getConfigurator();
    private TalonFXConfigurator pivotDrive_configurer = intakeDrive.getConfigurator();

    private final DutyCycleOut intakeDrive_Request = new DutyCycleOut(0.0); // CTRE pheonix 6 API
    private final MotionMagicConfigs intakeDrive_req = new MotionMagicConfigs();

    private double INTAKE_MAX_SPEED = 1;
    private double INTAKE_SLOW_SPEED = 0.75;

    private double OpenLoopSpeed = 0;

    private boolean OpenLoop = false;
        
    private enum IntakePosition{
        Zero(0),
        Passing(30),  //TODO: WILD GUESS
        Running(180); // TODO: WILD GUESS

        

        private final int setpoint; // enum contructer lest us assign values (enums are technically classes)
        private IntakePosition(int setpoint){
            this.setpoint = setpoint;
        }
    }

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

    public void IntakePart() {
        OpenLoopSpeed = .75;
        OpenLoopDriveIntake();        
    }

    public void SlowIntake() {
        OpenLoopSpeed = .15;
        OpenLoopDriveIntake();        
    }

    public void Eject() {
        OpenLoopSpeed = -.5;
        OpenLoopDriveIntake();
    }

    public void gotoSetpoint(IntakePosition pos){
        
    }

    @Override
    public void periodic(){
        if(OpenLoop){
            intakeDrive.setControl(intakeDrive_Request.withOutput(OpenLoopSpeed)); // go brr code?
            
        } else {
            intakeDrive.setControl(intakeDrive_Request.withOutput(0));
        }     
    }

    




}
