package frc.robot.subsystems.pose;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.trap.Trap;
import frc.robot.subsystems.trap.TrapPivot;
import frc.robot.subsystems.util.BSLogger;

public class ClimbPoseCommands {
    static Command startClimbPose() {
        return Commands.sequence(
                Commands.runOnce(() -> BSLogger.log("ClimbManager", "StartClimbPose")),
                Commands.parallel(
                        Subsystems.trap.moveToStateCmd(Trap.TrapState.Climb),
                        Commands.runOnce(Subsystems.shooter::stopShooter),
                        Commands.runOnce(Subsystems.shooter::stopFeeder),
                        Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.Up),
                        Subsystems.climber.moveToStateCmd(Climber.ClimberPosition.UP)),
                new WaitCommand(0.25),
                Subsystems.intake.moveToStateCmd(Intake.IntakeState.Climb));
    }

    public static Command rotateTrapArmsUp() {
        return Commands.sequence(
                Commands.runOnce(() -> BSLogger.log("ClimbManager", "rotateTrapArmsUp")),
                Commands.runOnce(() -> Subsystems.trap.getPivot().setTrapPosition(TrapPivot.TrapPivotPosition.Top)
                ));
    }

    public static Command pullUp() {
        return Commands.sequence(
                Commands.runOnce(() -> BSLogger.log("ClimbManager", "pullUp")),
                Subsystems.climber.moveToStateCmd(Climber.ClimberPosition.DOWN)
        );
    }

    public static Command putNoteInTrap() {
        return Commands.sequence(
                Commands.runOnce(() -> BSLogger.log("ClimbManager", "putNoteInTrap")),
                new WaitCommand(0.5),
                Subsystems.trap.moveToStateCmd(Trap.TrapState.ScoreFromClimb),
                new WaitCommand(0.5),
                Commands.runOnce(() -> Subsystems.trap.setFingerPosition(Trap.FingerPositions.Open)),
                new WaitCommand(0.5),
                Commands.runOnce(() -> Subsystems.trap.setFingerPosition(Trap.FingerPositions.Closed))
        );
    }

    public static Command clearTrapArms() {
        return Commands.runOnce(() -> Subsystems.trap.getPivot().setTrapPosition(TrapPivot.TrapPivotPosition.Top));
    }
}
