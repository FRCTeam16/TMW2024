package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems;
import frc.robot.commands.auto.WaitIntakeHasNoteCommand;
import frc.robot.commands.auto.WaitIntakeInPosition;
import frc.robot.commands.auto.WaitPivotInPosition;
import frc.robot.commands.auto.WaitShooterAtSpeed;
import frc.robot.subsystems.VisionAimManager;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.util.BSLogger;

public class ApplyShootingProfileCommand extends SequentialCommandGroup {
    public ApplyShootingProfileCommand(VisionAimManager.ShootingProfile profile) {
        addCommands(
                Commands.parallel(
                        Commands.runOnce(() -> BSLogger.log("ApplyShootingProfileCommand", "starting")),
                        new WaitIntakeHasNoteCommand(),
                        Commands.runOnce(() -> Subsystems.pivot.applyShootingProfile(profile), Subsystems.pivot),
                        Commands.runOnce(() -> Subsystems.shooter.applyShootingProfile(profile), Subsystems.shooter)
                ).withTimeout(1.0),
                Commands.parallel(
                        Commands.runOnce(() -> BSLogger.log("ApplyShootingProfileCommand", "waiting for position and speed")),
                        new WaitPivotInPosition(),
                        new WaitShooterAtSpeed(),
                        new WaitIntakeInPosition(IntakePivot.IntakePosition.Zero)
                ).withTimeout(1.0)
        );
    }
}
