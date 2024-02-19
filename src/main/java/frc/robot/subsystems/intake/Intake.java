package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.pose.PoseManager;

import java.util.Optional;


public class Intake extends SubsystemBase implements Lifecycle, Sendable {

    public static final String SUBSYSTEM_NAME = "IntakeSubsystem";
    private final IntakeSpeed intakeSpeed = new IntakeSpeed(new TalonFX(9));
    private final IntakePivot intakePivot = new IntakePivot(new TalonFX(10));
    private final DigitalInput noteDetector = new DigitalInput(0);

    private Optional<Timer> postNoteDetectedTimer = Optional.empty();


    public enum IntakeState {
        StartingPosition,
        IntakeFromFloor,
        HoldNote,
        AmpAim,
        Trap
    }

    private IntakeState intakeState = IntakeState.StartingPosition;


    /**
     * Initializes the Intake subsystem.
     */
    public Intake() {
    }

    @Override
    public void teleopInit() {
        intakeSpeed.teleopInit();
        intakePivot.teleopInit();
        ;
    }

    @Override
    public void autoInit() {
        intakeSpeed.autoInit();
        intakePivot.autoInit();
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
        DataLogManager.log("[Intake] setting intake state: " + state);
        switch (state) {
            case IntakeFromFloor -> {
                intakeSpeed.runIntakeFast();
                intakePivot.setIntakePosition(IntakePivot.IntakePosition.Pickup);
            }
            case HoldNote -> {
                intakeSpeed.stopIntake();
                intakePivot.setIntakePosition(IntakePivot.IntakePosition.Vertical);
            }
            default -> {
                DataLogManager.log("[Intake] Unhandled IntakeState: " + state.name());
            }
        }

        this.intakeState = state;
    }

    @Override
    public void periodic() {
        if (IntakeState.IntakeFromFloor == intakeState && isNoteDetected()) {
            Subsystems.poseManager.schedulePose(PoseManager.Pose.NotePickedUp);
            postNoteDetectedTimer = Optional.of(new Timer());
            postNoteDetectedTimer.get().start();
        }

        // Check if we should override intake speed
        if (postNoteDetectedTimer.isPresent() &&  postNoteDetectedTimer.get().hasElapsed(0.5)) {
            intakeSpeed.stopIntake();
            postNoteDetectedTimer.get().stop();
            postNoteDetectedTimer = Optional.empty();   // todo cleanup?
        }

        intakeSpeed.periodic();
        intakePivot.periodic();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(SUBSYSTEM_NAME);
        builder.addStringProperty("IntakeState", () -> this.intakeState.name(), null);
        builder.addBooleanProperty("NoteDetected", this::isNoteDetected, null);
        intakeSpeed.initSendable(builder);
        intakePivot.initSendable(builder);
    }

    public Command moveToStateCmd(IntakeState state) {
        return Commands.runOnce(() -> this.setIntakeState(state));
    }

}
