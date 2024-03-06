package frc.robot.subsystems.trap;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Lifecycle;

public class Trap extends SubsystemBase implements Lifecycle, Sendable {
    public static final String SUBSYSTEM_NAME = "TrapSubsystem";
    private final TrapExtender extender = new TrapExtender(new TalonFX(21));
    private final TrapPivot pivot = new TrapPivot(new TalonFX(22));
    private final Servo fingerLeft = new Servo(0);
    private final Servo fingerRight = new Servo(1);

    private TrapState state = TrapState.Default;

    public enum TrapState {
        Default,
        FeedNoteToTrap,
        Drive,
        PivotDrive,
        AmpShotExtend,
        AmpShotPivot,
        AmpShotDeflectShot,

        Climb,
        ScoreFromClimb, ;
    }

    @Override
    public void teleopInit() {
        this.getExtender().teleopInit();
        this.getPivot().teleopInit();
        setFingerPosition(FingerPositions.StartPosition);
    }

    @Override
    public void autoInit() {
        this.getExtender().autoInit();
        this.getPivot().autoInit();
    }

    public Command moveToStateCmd(TrapState state) {
        return Commands.runOnce(() -> setTrapState(state));
    }

    public TrapState getTrapState() {
        return state;
    }


    public void setTrapState(TrapState state) {
        this.state = state;
        switch (state) {
            case Default:
                getPivot().setTrapPosition(TrapPivot.TrapPivotPosition.Drive);
                getExtender().setTrapPosition(TrapExtender.TrapPosition.Zero);
                setFingerPosition(FingerPositions.StartPosition);
                break;
            case Drive:
                getPivot().setTrapPosition(TrapPivot.TrapPivotPosition.Drive);
                getExtender().setTrapPosition(TrapExtender.TrapPosition.Zero);
                setFingerPosition(FingerPositions.Closed);
                break;
            case PivotDrive:
                getPivot().setTrapPosition(TrapPivot.TrapPivotPosition.Drive);
                break;
            case FeedNoteToTrap:
                getPivot().setTrapPosition(TrapPivot.TrapPivotPosition.PreClimb);
                getExtender().setTrapPosition(TrapExtender.TrapPosition.Zero);
                setFingerPosition(FingerPositions.Closed);
                break;
            case AmpShotExtend:
                getExtender().setTrapPosition(TrapExtender.TrapPosition.AmpShot);
                break;
            case AmpShotPivot:
                getPivot().setTrapPosition(TrapPivot.TrapPivotPosition.AmpShot);
                break;
            case AmpShotDeflectShot:
                getPivot().setTrapPosition(TrapPivot.TrapPivotPosition.AmpDeflect);
                break;
            case Climb:
                getPivot().setTrapPosition(TrapPivot.TrapPivotPosition.PreClimb);
                getExtender().setTrapPosition(TrapExtender.TrapPosition.Zero);
                break;
            case ScoreFromClimb:
                getPivot().setTrapPosition(TrapPivot.TrapPivotPosition.Score);
                break;
        }
    }

    public void setFingerPosition(FingerPosition position) {
        this.fingerLeft.setPosition(position.leftPosition);
        this.fingerRight.setPosition(position.rightPosition);
    }

    public TrapExtender getExtender() {
        return extender;
    }

    public TrapPivot getPivot() {
        return pivot;
    }

    @Override
    public void periodic() {
        extender.periodic();
        pivot.periodic();
    }

    public record FingerPosition(double leftPosition, double rightPosition) {
    }

    public static class FingerPositions {
        public static final FingerPosition StartPosition = new FingerPosition(0.85, 0.15);
        public static final FingerPosition Closed = new FingerPosition(0.85, 0.15);
        public static final FingerPosition Open = new FingerPosition(0.5, 0.5);
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        if (Constants.Dashboard.UseSendables) {
            builder.setSmartDashboardType(SUBSYSTEM_NAME);
            // TODO: expose finger values
            pivot.initSendable(builder);
            extender.initSendable(builder);

            builder.addDoubleProperty("Servo/finger1Angle", fingerLeft::getAngle, fingerLeft::setAngle);
            builder.addDoubleProperty("Servo/finger2Angle", fingerRight::getAngle, fingerRight::setAngle);

            SmartDashboard.putData("TrapSubsystem/Debug/FL", fingerLeft);
            SmartDashboard.putData("TrapSubsystem/Debug/FR", fingerRight);
        }
    }
}

