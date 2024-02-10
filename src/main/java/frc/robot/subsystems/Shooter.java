package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase implements Lifecycle{
    private TalonFX beltDrive_1 = new TalonFX(25000); // give real number
    private TalonFX beltDrive_2 = new TalonFX(25001); // give real number

    private TalonFX noteFeederDrive = new TalonFX(25002); // give real number
    private TalonFX aimDrive = new TalonFX(25003); // give real numuber


    // the configs ( default by default )
    private TalonFXConfigurator beltDriveConfigurer = beltDrive_1.getConfigurator();
    private TalonFXConfigurator feederConfigurer = noteFeederDrive.getConfigurator();
    private TalonFXConfigurator aimDriveConfigurer = aimDrive.getConfigurator();

    private TalonFXConfiguration defaultConfig = new TalonFXConfiguration();


    // the request things
    private final DutyCycleOut beltDrive_Request = new DutyCycleOut(0.0);
    private final DutyCycleOut feederDrive_Request = new DutyCycleOut(0.0);
    private final MotionMagicDutyCycle aimDrive_Request = new MotionMagicDutyCycle(0.0);
    
    private aimPosition currenPosition = aimPosition.Zero;
    private boolean shooterOpenLoop = false;
    private boolean feederOpenLoop = false;

    private double feederOpenLoopSpeed = 0.0;
    private double shooterOpenLoopSpeed = 0.0;

    private boolean positionChanged = false;

    public enum aimPosition{ 
        Zero(0),
        sampleText(100);  // sample text

        final int setPoint;
        private aimPosition(int i){
            this.setPoint = i;
        }
    }

    public Shooter(){
        aimDriveConfigurer.apply(defaultConfig);
        feederConfigurer.apply(defaultConfig);
        beltDriveConfigurer.apply(defaultConfig);

        SmartDashboard.setDefaultNumber("Shooter/beltFastSpeed", 0.75); // make editable from Elastic
        SmartDashboard.setDefaultNumber("Shooter/beltSlowSpeed", 0.25);
        SmartDashboard.setDefaultNumber("Shooter/feederFastSpeed", 0.50);
    }


    public void runFeederOpenLoop(){
        feederOpenLoop = true;
    }

    public void stopFeederOpenLoop(){
        feederOpenLoop = false;
    }

    public void runShooterOpenLoop(){
        shooterOpenLoop = true;
    }

    public void stopShooterOpenLoop(){
        shooterOpenLoop = false;
    }

    public void slowFeed(){
        feederOpenLoopSpeed = SmartDashboard.getNumber("Shooter/feederFastSpeed", 0.50);
        runFeederOpenLoop();
    }

    public void fastShoot(){
        shooterOpenLoopSpeed = SmartDashboard.getNumber("Shooter/beltSlowSpeed", 0.75);
        runShooterOpenLoop();
    }

    public void slowShoot(){
        shooterOpenLoopSpeed = SmartDashboard.getNumber("Shooter/beltSlowSpeed", 0.25);
        runShooterOpenLoop();
    }

    @Override
    public void periodic(){
        if(feederOpenLoop){
            noteFeederDrive.setControl(feederDrive_Request.withOutput(feederOpenLoopSpeed));
        }
        if(shooterOpenLoop){
            beltDrive_1.setControl(beltDrive_Request.withOutput(shooterOpenLoopSpeed));
            beltDrive_2.setControl(beltDrive_Request.withOutput(shooterOpenLoopSpeed));
        }
        if(positionChanged){
            // update position
            // TODO add constraints using .withLimitForward etc.
            aimDrive.setControl(aimDrive_Request.withPosition(currenPosition.setPoint));
            positionChanged = false;
        }
    }
}
