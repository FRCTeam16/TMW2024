package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.util.PIDHelper;

public class ShooterPrototype extends SubsystemBase implements Lifecycle {
    private CANSparkMax motor1 = new CANSparkMax(31, MotorType.kBrushless);
    private CANSparkMax motor2 = new CANSparkMax(32, MotorType.kBrushless);

    private ControlMode controlMode = ControlMode.OpenLoop;
    private boolean motor1Enabled = false;
    private boolean motor2Enabled = false;

    private PIDHelper pidHelper1 = new PIDHelper("ShooterPrototype/Motor1PID");
    private PIDHelper pidHelper2 = new PIDHelper("ShooterPrototype/Motor2PID");
    private boolean closedLoopEnabled = false;

    private enum ControlMode {
        OpenLoop,
        ClosedLoop
    };

    public ShooterPrototype() {
        motor1.restoreFactoryDefaults();
        motor2.restoreFactoryDefaults();

        motor1.setOpenLoopRampRate(1.0);
        motor2.setOpenLoopRampRate(1.0);

        SmartDashboard.setDefaultNumber("ShooterPrototype/OpenLoop1", 0);
        SmartDashboard.setDefaultNumber("ShooterPrototype/OpenLoop2", 0);

        pidHelper1.initialize(0, 0, 0, 0, 0, 0);
        pidHelper2.initialize(0, 0, 0, 0, 0, 0);
    }

    @Override
    public void teleopInit() {
        this.motor1Enabled = false;
        this.motor2Enabled = false;
        this.closedLoopEnabled = false;
    }
    
    public void enableMotor1OpenLoop() {
        this.controlMode = ControlMode.OpenLoop;
        this.motor1Enabled = true;
    }

    public void disableMotor1OpenLoop() {
        this.motor1Enabled = false;
    }

    public void disableMotor2OpenLoop() {
        this.motor2Enabled = false;
    }

    public void enableMotor2OpenLoop() {
        this.controlMode = ControlMode.OpenLoop;
        this.motor2Enabled = true;
    }

    public void toggleClosedLoop() {
        this.controlMode = ControlMode.ClosedLoop;
        this.closedLoopEnabled = !this.closedLoopEnabled;
    }

    public void toggleOpenLoop() {
        if (this.motor1Enabled) {
            this.disableMotor1OpenLoop();
            this.disableMotor2OpenLoop();
        } else {
            this.enableMotor1OpenLoop();
            this.enableMotor2OpenLoop();
            this.closedLoopEnabled = false;
        }
    }

    public void disableOpenLoop() {
        this.disableMotor1OpenLoop();
        this.disableMotor2OpenLoop();
    }


    @Override
    public void periodic() {
        // System.out.println("SHooterPrototype periodic");
        if (ControlMode.OpenLoop == controlMode) {
            double m1Speed = SmartDashboard.getNumber("ShooterPrototype/OpenLoop1", 0);
            double m2Speed = SmartDashboard.getNumber("ShooterPrototype/OpenLoop2", 0);
            if (motor1Enabled) {
                motor1.set(m1Speed);
            } else {
                motor1.set(0.0);
            }
            if (motor2Enabled) {
                motor2.set(m2Speed);
            } else {
                motor2.set(0.0);
            }
        } else {
            pidHelper1.updateValuesFromDashboard();
            pidHelper2.updateValuesFromDashboard();

            var pid1 = motor1.getPIDController();
            pid1.setP(pidHelper1.kP);
            pid1.setI(pidHelper1.kI);
            pid1.setD(pidHelper1.kD);

            var pid2 = motor2.getPIDController();
            pid2.setP(pidHelper1.kP);
            pid2.setI(pidHelper1.kI);
            pid2.setD(pidHelper1.kD);

            if (closedLoopEnabled) {
                pid1.setReference(pidHelper1.kV, ControlType.kVelocity);
                pid2.setReference(pidHelper2.kV, ControlType.kVelocity);
            } else {
                pid1.setReference(0, ControlType.kVelocity);
                pid2.setReference(0, ControlType.kVelocity);
            }
        }

    }
    
}

