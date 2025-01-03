package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.intake.IntakePivot.IntakePosition;
import frc.robot.subsystems.pose.PoseManager;
import frc.robot.subsystems.util.BSLogger;

import java.util.Optional;


public class Intake extends SubsystemBase implements Lifecycle, Sendable {

    public static final String SUBSYSTEM_NAME = "IntakeSubsystem";
    private final IntakeSpeed intakeSpeed = new IntakeSpeed(new TalonFX(9));
    private final IntakePivot intakePivot = new IntakePivot(new TalonFX(10));
    private final DigitalInput noteDetector = new DigitalInput(0);
    private boolean firstPartOfTeleop = false; 
    // starts in false state, then changes to true through initTeleop then when sensor triggers, run shoot and turn off
    // effectivly, it is whether or not first detect start should happen at any point in time

    private Optional<Timer> postNoteDetectedTimer = Optional.empty();

    private Optional<Timer> ampShotTimer = Optional.empty();

    private static final double POST_NOTE_DETECT_TIME = 0.15;




    public enum IntakeState {
        StartingPosition,
        IntakeFromFloor,
        MoveIntakeToFloorWithoutIntaking,
        HoldNote,
        FeedNote,

        StopFeed,


        RotateUpWhileFeedingNote,
        Climb,

        AmpAim,
        TryClearNote,
        TryShootAmp, 
        ReverseFeed
    }

    private IntakeState intakeState = IntakeState.StartingPosition;


    /**
     * Initializes the Intake subsystem.
     */
    public Intake() {
        var cfg = intakePivot.getMotionMagicConfigs();
        this.original_velocity = cfg.MotionMagicCruiseVelocity;
    }

    @Override
    public void teleopInit() {
        firstPartOfTeleop = true;
        intakeSpeed.teleopInit();
        intakePivot.teleopInit();
    }

    @Override
    public void autoInit() {
        intakeSpeed.autoInit();
        intakePivot.autoInit();
        intakeState = IntakeState.StartingPosition;
    }

    public boolean isNoteDetected() {
        return !noteDetector.get();
    }

    public IntakeSpeed getIntakeSpeed() {
        return intakeSpeed;
    }

    public IntakePivot getIntakePivot() {
        return intakePivot;
    }

    public void setIntakeState(IntakeState state) {
        BSLogger.log("Intake", "setting intake state: " + state);
        switch (state) {
            case StartingPosition, HoldNote -> {
                intakeSpeed.stopIntake();
                intakePivot.setIntakePosition(IntakePivot.IntakePosition.Zero);
            }

            case MoveIntakeToFloorWithoutIntaking -> {
                intakePivot.setIntakePosition(IntakePivot.IntakePosition.Pickup);
            }

            case IntakeFromFloor -> {
                intakeSpeed.runIntakeFast();
                intakePivot.setIntakePosition(IntakePivot.IntakePosition.Pickup);
            }
            case RotateUpWhileFeedingNote -> {
                intakePivot.setIntakePosition(IntakePivot.IntakePosition.Zero);
            }
            case FeedNote -> {
                intakeSpeed.runIntakeFeedShooter();
            }
            case AmpAim -> {
                intakePivot.setIntakePosition(IntakePivot.IntakePosition.MotorPreAMPShot);
            }
            case TryShootAmp -> {
                intakePivot.setIntakePosition(IntakePivot.IntakePosition.AMPShot);
            }
            case Climb -> {
                intakePivot.setIntakePosition(IntakePivot.IntakePosition.Climb);
                intakeSpeed.stopIntake();
            }
            case StopFeed -> {
                intakeSpeed.stopIntake();
            }
            case TryClearNote -> {
                intakeSpeed.runIntakeEject();
                intakePivot.setIntakePosition(IntakePivot.IntakePosition.Pickup);
            }
            case ReverseFeed -> {
                intakeSpeed.runIntakeFast();
                intakePivot.setIntakePosition(IntakePosition.Zero);
            }
            default -> {
                BSLogger.log("Intake", "Unhandled IntakeState: " + state.name());
            }
        }

        this.intakeState = state;
    }


    public Command resetEncoderCmd() {
        return this.runOnce(this.intakePivot::resetEncoder);
    }

    public IntakeState getIntakeState() {
        return intakeState;
    }

    public boolean isInPosition(IntakePivot.IntakePosition position) {
        return intakePivot.isInPosition(position);
    }

    @Override
    public void periodic() {

        if (IntakeState.IntakeFromFloor == intakeState && isNoteDetected()) {
            postNoteDetectedTimer = Optional.of(new Timer());
            postNoteDetectedTimer.get().start();
            this.setIntakeState(IntakeState.RotateUpWhileFeedingNote);
        }

        // Check if we should override intake speed
        if (postNoteDetectedTimer.isPresent() &&  postNoteDetectedTimer.get().hasElapsed(POST_NOTE_DETECT_TIME)) {
            intakeSpeed.stopIntake();
            postNoteDetectedTimer.get().stop();
            postNoteDetectedTimer = Optional.empty();   // todo determine if optional<> approach is clearest
            setIntakeState(IntakeState.HoldNote);
        }

        //
        // Trying Amp Shot
        //
        if (ampShotTimer.isPresent() && ampShotTimer.get().hasElapsed(0.25)) {
            ampShotTimer = Optional.empty();
            setIntakeState(IntakeState.StopFeed);
        } else if (IntakeState.TryShootAmp == intakeState && ampShotTimer.isEmpty()) {
            ampShotTimer = Optional.of(new Timer());
            ampShotTimer.get().start();
            intakeSpeed.runAmpShot();
        }

        if(isNoteDetected() && firstPartOfTeleop){
            Subsystems.shooter.runShooter(); // delay shooter start untill part detected
            firstPartOfTeleop = false;
        }

        intakeSpeed.periodic();
        intakePivot.periodic();
    }



    private Optional<Timer> tryShootAmp = Optional.empty();
    private double ampShotTime = 0.5;

    private double shotPosition = -4.5;

    private double original_velocity = 0;
    private double MM_Velocity = 12.0;

    public double getAmpShotTime() {
        return ampShotTime;
    }

    public void setAmpShotTime(double ampShotTime) {
        this.ampShotTime = ampShotTime;
    }

    public double getShotPosition() {
        return shotPosition;
    }

    public void setShotPosition(double shotPosition) {
        this.shotPosition = shotPosition;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        if (Constants.Dashboard.UseSendables) {
            builder.setSmartDashboardType(SUBSYSTEM_NAME);
            builder.addStringProperty("IntakeState", () -> this.intakeState.name(), null);
            builder.addBooleanProperty("NoteDetected", this::isNoteDetected, null);

            intakeSpeed.initSendable(builder);
            intakePivot.initSendable(builder);

            if (Constants.Dashboard.ConfigurationMode && Constants.Dashboard.IntakeConfigMode) {
                builder.addDoubleProperty("AmpShotWait", this::getAmpShotTime, this::setAmpShotTime);
                builder.addDoubleProperty("AmpShotPosition", this::getShotPosition, this::setShotPosition);
                builder.addDoubleProperty("MM_V", () -> this.MM_Velocity, (v) -> this.MM_Velocity = v);
            }
        }
    }


    public Command moveToStateCmd(IntakeState state) {
        return Commands.runOnce(() -> this.setIntakeState(state), Subsystems.intake);
    }

}
