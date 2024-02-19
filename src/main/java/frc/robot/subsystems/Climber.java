package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.util.PIDHelper;

public class Climber extends SubsystemBase implements Lifecycle, Sendable {
  public static final String SUBSYSTEM_NAME = "ClimberSubsystem";
  // motor configuration
  private final TalonFX climberDrive = new TalonFX(35);// set to 125:1
  private final DutyCycleOut openLoopOut = new DutyCycleOut(0).withEnableFOC(true);
  private final MotionMagicVoltage motionMagicOut = new MotionMagicVoltage(0);

  private final PIDHelper pidHelper = new PIDHelper(SUBSYSTEM_NAME + "/PID");

  //
  // Open Loop Handling
  //
  private boolean openLoop = false;
  private double openLoopSpeed = 0.0;
  private final ClimberOpenLoopSpeeds climberOpenLoopSpeeds = new ClimberOpenLoopSpeeds();

  public enum ClimberPosition {
    Stow(0),
    up(100);

    public final double setpoint;

    private ClimberPosition(double setpoint) {
      this.setpoint = setpoint;
    }
  }

  private ClimberPosition currentPosition = ClimberPosition.Stow;

  // Closed loop setpoint
  private double setpoint = 0.0;

  public Climber() {
//    climberDrive.setNeutralMode(NeutralModeValue.Brake);
    pidHelper.initialize(0, 0, 0, 0, 0, 0);
    TalonFXConfiguration config = new TalonFXConfiguration();
    pidHelper.updateConfiguration(config.Slot0);
    climberDrive.getConfigurator().apply(config);
    setpoint = climberDrive.getPosition().getValue();
  }

  @Override
  public void teleopInit() {
    setpoint = climberDrive.getPosition().getValue();
  }

  @Override
  public void autoInit() {
    setpoint = climberDrive.getPosition().getValue();

  }

  public boolean isOpenLoop() {
    return openLoop;
  }

  public void setOpenLoop(boolean openLoop) {
    this.openLoop = openLoop;
  }

  public void openLoopUp() {
    openLoop = true;
    openLoopSpeed = climberOpenLoopSpeeds.getDownSpeed();
  }

  public void openLoopDown() {
    openLoop = true;
    openLoopSpeed = climberOpenLoopSpeeds.getDownSpeed();
  }

  public void stopOpenLoop() {
    openLoop = true;
    openLoopSpeed = 0.0;
  }

  public void setClimberPosition(ClimberPosition position) {
    this.currentPosition = position;
    this.setClimberSetpoint(position.setpoint);
  }

  public ClimberPosition getClimberPosition() {
    return this.currentPosition;
  }

  private void setClimberSetpoint(double setpoint) {
    openLoop = false;
    this.setpoint = setpoint;
  }

  public double getSetpoint() {
    return setpoint;
  }

  public void setSetpoint(double setpoint) {
    this.setpoint = setpoint;
  }

  @Override
  public void periodic() {
    if (openLoop) {
      climberDrive.setControl(openLoopOut.withOutput(openLoopSpeed));
    } else {
      climberDrive.setControl(motionMagicOut.withPosition(setpoint));
    }
  }

  static class ClimberOpenLoopSpeeds {
    private double upSpeed = 0.1;
    private double downSpeed = -0.1;

    public double getUpSpeed() {
      return upSpeed;
    }

    public void setUpSpeed(double upSpeed) {
      this.upSpeed = upSpeed;
    }

    public double getDownSpeed() {
        return downSpeed;
    }

    public void setDownSpeed(double downSpeed) {
        this.downSpeed = downSpeed;
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType(SUBSYSTEM_NAME);
    builder.addDoubleProperty("OpenLoop/UpSpeed", this.climberOpenLoopSpeeds::getUpSpeed, this.climberOpenLoopSpeeds::setUpSpeed);
    builder.addDoubleProperty("OpenLoop/DownSpeed", this.climberOpenLoopSpeeds::getDownSpeed, this.climberOpenLoopSpeeds::setDownSpeed);
    builder.addDoubleProperty("OpenLoop/Speed", () -> this.openLoopSpeed, null);

    builder.addDoubleProperty("Setpoint", this::getSetpoint, this::setSetpoint);
    builder.addStringProperty("ClimberPosition", () -> this.getClimberPosition().name(), null);
    builder.addDoubleProperty("Encoder", () -> this.climberDrive.getPosition().getValue(), null);
    builder.addDoubleProperty("Velocity", () -> this.climberDrive.getVelocity().getValue(), null);
    builder.addDoubleProperty("Acceleration", () -> this.climberDrive.getAcceleration().getValue(), null);
  }
}