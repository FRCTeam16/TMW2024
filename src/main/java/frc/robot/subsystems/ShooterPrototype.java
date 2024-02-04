/*package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.util.PIDHelper;

public class ShooterPrototype extends SubsystemBase implements Lifecycle {
    private CANSparkMax motor1 = new CANSparkMax(31, MotorType.kBrushless);
    private CANSparkMax motor2 = new CANSparkMax(32, MotorType.kBrushless);

    private Servo finger1 = new Servo(1);
    private Servo finger2 = new Servo(2);


    private ControlMode controlMode = ControlMode.OpenLoop;
    private boolean motor1Enabled = false;
    private boolean motor2Enabled = false;

    private PIDHelper pidHelper1 = new PIDHelper("ShooterPrototype/Motor1PID");
    private PIDHelper pidHelper2 = new PIDHelper("ShooterPrototype/Motor2PID");
    private boolean closedLoopEnabled = false;


    private boolean oneShot = true;

    private enum ControlMode {
        OpenLoop,
        ClosedLoop
    };

    public ShooterPrototype() {
        motor1.restoreFactoryDefaults();
        motor2.restoreFactoryDefaults();

        motor1.setOpenLoopRampRate(1.0);
        motor1.setClosedLoopRampRate(1.0);
        motor2.setOpenLoopRampRate(1.0);
        motor2.setClosedLoopRampRate(1.0);

        SmartDashboard.setDefaultNumber("ShooterPrototype/OpenLoop1", 0);
        SmartDashboard.setDefaultNumber("ShooterPrototype/OpenLoop2", 0);

        SmartDashboard.setDefaultNumber("ShooterPrototype/m1Setpoint", 0);
        SmartDashboard.setDefaultNumber("ShooterPrototype/m2Setpoint", 0);

        pidHelper1.initialize(0.00005, 0, 0, 0.00018, 0, 0);
        pidHelper2.initialize(0.00005, 0, 0, 0.00018, 0, 0);

        SmartDashboard.setDefaultNumber("Prototype/Finger1Angle", 0);
        SmartDashboard.setDefaultNumber("Prototype/Finger2Angle", 0);
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
        controlServo();
        SmartDashboard.putBoolean("ShooterPrototype/M1Enabled", this.motor1Enabled);
        SmartDashboard.putBoolean("ShooterPrototype/M2Enabled", this.motor2Enabled);
        SmartDashboard.putBoolean("ShooterPrototype/closedLoopEnabled", this.closedLoopEnabled);

        SmartDashboard.putNumber("ShooterPrototype/M1Velocity", this.motor1.getEncoder().getVelocity());

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
            // pid1.setFF(0);
            if (oneShot) {
            pid1.setP(pidHelper1.kP);
            pid1.setI(pidHelper1.kI);
            pid1.setD(pidHelper1.kD);
            pid1.setFF(pidHelper1.kF);
            }

            var pid2 = motor2.getPIDController();
            // pid2.setFF(0);
            if (oneShot) {
            pid2.setP(pidHelper2.kP);
            pid2.setI(pidHelper2.kI);
            pid2.setD(pidHelper2.kD);
            pid2.setFF(pidHelper2.kF);
            }

            // if (oneShot) { oneShot = false; }

            if (closedLoopEnabled) {
                double m1Setpoint = SmartDashboard.getNumber("ShooterPrototype/m1Setpoint", 0);
                double m2Setpoint = SmartDashboard.getNumber("ShooterPrototype/m2Setpoint", 0);
                pid1.setReference(m1Setpoint, ControlType.kVelocity);
                pid2.setReference(m2Setpoint, ControlType.kVelocity);
            } else {
                // System.out.println("ShooterProrotype - disabled closed loop");
                pid1.setReference(0, ControlType.kVelocity);
                pid2.setReference(0, ControlType.kVelocity);
            }
        }
 
    }

    private void controlServo() {
        finger1.set(SmartDashboard.getNumber("Prototype/Finger1Angle", 0));
        finger2.set(SmartDashboard.getNumber("Prototype/Finger2Angle", 0));
    }
    
}


