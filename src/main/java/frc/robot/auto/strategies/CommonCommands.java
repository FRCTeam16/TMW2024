package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.auto.*;
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
                AutoPathStrategy.writeLog("DoShotCommand", "Using profile: " + profile),
                //
                Commands.sequence(
                        Commands.parallel(
                                AutoPathStrategy.writeLog("DoShotCommand", "Setting shooter and pivot profiles"),
                                new EnableShooterCommand().unless(() -> Subsystems.shooter.isEnabled()),
                                Commands.runOnce(() -> Subsystems.shooter.applyShootingProfile(profile)),
                                Commands.runOnce(() -> Subsystems.pivot.applyShootingProfile(profile))
                        ),
                        Commands.parallel(
                                new WaitPivotInPosition(),
                                new WaitShooterAtSpeed()
                        ).withTimeout(0.5),
                        AutoPathStrategy.writeLog("DoShotCommand", "******* WILL BE FIRING NEXT *******"),
//                        new WaitCommand(0.5),
                        CommonCommands.doShootCmd()
                ),
                Subsystems.poseManager.getPoseCommand(
                        name == "TabFAS3" ? PoseManager.Pose.PickupNoTrap : PoseManager.Pose.Pickup)
        ).beforeStarting(() -> BSLogger.log("DoShotCommand", "starting: " + name))
                .finallyDo(() -> BSLogger.log("DoShotCommand", "ending: " + name));
    }

    public static Command DoTabShot2Command() {
        String name = "TabShot2";
        VisionAimManager.ShootingProfile profile = Tab.SecondShot;
        return new SequentialCommandGroup(
                // Start point is that we expect the shooter to have a note
                new WaitShooterHasNote().withTimeout(0.5),
                Commands.parallel(
                        Subsystems.intake.moveToStateCmd(Intake.IntakeState.MoveIntakeToFloorWithoutIntaking),
                        AutoPathStrategy.writeLog("DoShotCommand", "Using profile: " + profile)
                ),
                //
                Commands.sequence(
                        Commands.parallel(
                                AutoPathStrategy.writeLog("DoShotCommand", "Setting shooter and pivot profiles"),
                                new EnableShooterCommand().unless(() -> Subsystems.shooter.isEnabled()),
                                Commands.runOnce(() -> Subsystems.shooter.applyShootingProfile(profile)),
                                Commands.runOnce(() -> Subsystems.pivot.applyShootingProfile(profile))
                        ),
                        Commands.parallel(
                                new WaitPivotInPosition(),
                                new WaitShooterAtSpeed()
                        ).withTimeout(0.5),
                        AutoPathStrategy.writeLog("DoShotCommand", "******* WILL BE FIRING NEXT *******"),
//                        new WaitCommand(0.5),
                        CommonCommands.doShootCmd() // has 0.2 delay
                ),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Pickup),
                Commands.runOnce((() -> Subsystems.pivot.applyShootingProfile(Tab.ThirdShot)))  // override pose
        ).beforeStarting(() -> BSLogger.log("DoTabShot2Command", "starting: " + name))
                .finallyDo(() -> BSLogger.log("DoTabShot2Command", "ending: " + name));
    }

    public static Command DoBlazeShotCommand(VisionAimManager.ShootingProfile profile) {
        return new SequentialCommandGroup(
                // Start point is that we expect the shooter to have a note
                new WaitShooterHasNote().withTimeout(0.5),
                AutoPathStrategy.writeLog("DoShotCommand", "If shooter has note then we will try shot"),
                Commands.sequence(
                        new EnableShooterCommand(),
                        Commands.parallel(
                                AutoPathStrategy.writeLog("DoShotCommand", "Setting shooter and pivot profiles"),
                                Commands.runOnce(() -> Subsystems.shooter.applyShootingProfile(profile)),
                                Commands.runOnce(() -> Subsystems.pivot.applyShootingProfile(profile))
                        ),
                        Commands.parallel(
                                new WaitPivotInPosition(),
                                new WaitShooterAtSpeed(),
                                new WaitCommand(0.5)
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

    public static Command feedAndShootAutoCmd(VisionAimManager.ShootingProfile profile) {
        return new FeedAndShootAuto(profile);
    }

    public static Command queueTabShot2() {
        return Commands.sequence(
                new EnableShooterCommand(),
                Commands.parallel(
                        Commands.runOnce(() -> Subsystems.shooter.applyShootingProfile(Tab.SecondShot), Subsystems.shooter)
                )
        );
    }

    public static Command queueTabShot4() {
        return Commands.sequence(
                new EnableShooterCommand(),
                Commands.parallel(
                        Commands.runOnce(() -> Subsystems.shooter.applyShootingProfile(Tab.ThirdShot), Subsystems.shooter),
                        Subsystems.pivot.queueNextProfileCommand(Tab.ThirdShot)
                )
        );
    }
}
