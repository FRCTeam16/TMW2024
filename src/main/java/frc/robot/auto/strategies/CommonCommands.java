package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.auto.EnableShooterCommand;
import frc.robot.commands.auto.WaitPivotInPosition;
import frc.robot.commands.auto.WaitShooterAtSpeed;
import frc.robot.commands.auto.WaitShooterHasNote;
import frc.robot.subsystems.VisionAimManager;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pose.PoseManager;
import frc.robot.subsystems.util.BSLogger;

public class CommonCommands {
    public static Command DoShotCommand(VisionAimManager.ShootingProfile profile) {
        return DoShotCommand("Default", profile);
    }
    public static Command DoShotCommand(String name, VisionAimManager.ShootingProfile profile) {
        return new SequentialCommandGroup(
                // Start point is that we expect the shooter to have a note
                new WaitShooterHasNote().withTimeout(0.5),
                Subsystems.intake.moveToStateCmd(Intake.IntakeState.MoveIntakeToFloorWithoutIntaking),
                AutoPathStrategy.writeLog("DoShotCommand", "If shooter has note then we will try shot"),
                //
                Commands.sequence(
                        Commands.parallel(
                                AutoPathStrategy.writeLog("DoShotCommand", "Setting shooter and pivot profiles"),
                                new EnableShooterCommand(),
                                Commands.runOnce(() -> Subsystems.shooter.applyShootingProfile(profile)),
                                Commands.runOnce(() -> Subsystems.pivot.applyShootingProfile(profile))
                        ),
                        Commands.parallel(
                                new WaitPivotInPosition(),
                                new WaitShooterAtSpeed(),
                                new WaitCommand(0.25)
                        ).withTimeout(0.5),
                        AutoPathStrategy.writeLog("DoShotCommand", "******* WILL BE FIRING NEXT *******"),
//                        new WaitCommand(0.5),
                        CommonCommands.doShootCmd()
                ).unless(() -> !Subsystems.shooter.isNoteDetected()),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Pickup)
        ).beforeStarting(() -> BSLogger.log("DoShotCommand", "starting: " + name))
                .finallyDo(() -> BSLogger.log("DoShotCommand", "ending: " + name));
    }

    public static Command DoBlazeShotCommand(VisionAimManager.ShootingProfile profile) {
        return new SequentialCommandGroup(
                // Start point is that we expect the shooter to have a note
                new WaitShooterHasNote().withTimeout(0.5),
                AutoPathStrategy.writeLog("DoShotCommand", "If shooter has note then we will try shot"),
                Commands.sequence(
                        Commands.parallel(
                                AutoPathStrategy.writeLog("DoShotCommand", "Setting shooter and pivot profiles"),
                                new EnableShooterCommand(),
                                Commands.runOnce(() -> Subsystems.shooter.applyShootingProfile(profile)),
                                Commands.runOnce(() -> Subsystems.pivot.applyShootingProfile(profile))
                        ),
                        Commands.parallel(
                            new WaitPivotInPosition(),
                            new WaitShooterAtSpeed(),
                            new WaitCommand(0.25)
                        ).withTimeout(0.5),
                        AutoPathStrategy.writeLog("DoShotCommand", "******* WILL BE FIRING NEXT *******"),
//                        new WaitCommand(0.5),
                        CommonCommands.doShootCmd()
                ).unless(() -> !Subsystems.shooter.isNoteDetected())
        ).beforeStarting(() -> BSLogger.log("DoShotCommand", "starting"))
                .finallyDo(() -> BSLogger.log("DoShotCommand", "ending"));
    }

    static Command doShootCmd() {
        return Commands.parallel(
                Commands.runOnce(() -> BSLogger.log("doShootCmd", "shooting")),
                Subsystems.shooter.shootCmd()
        );
    }
}
