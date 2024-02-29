package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.auto.PathRegistry;
import frc.robot.commands.auto.*;
import frc.robot.subsystems.VisionAimManager;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pose.PoseManager;
import frc.robot.subsystems.util.BSLogger;

public class DropShot extends AutoPathStrategy {

    private final VisionAimManager.ShootingProfile shotProfile = new VisionAimManager.ShootingProfile(26.5, 45, 32);
    public DropShot() {
        addCommands(
                Commands.parallel(
                        new PrintStartInfo("DropShot"),
                        new InitializeAutoState(0),
                        new EnableShooterCommand(true)
//                        Subsystems.swerveSubsystem.applyRequest(() -> pointWheels)
                ),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.PrepareBloopShot),
                Commands.runOnce(() -> BSLogger.log("DropShot", "Running")),
                this.runAutoPath("DropShot"),

                // Select which route to run
                Commands.either(
                        ds2RouteCmd(),
                        Commands.print("********************** Skipping DS2"),
                        () -> {
                            BSLogger.log("DropShot", "********************** NOTE DETECTED: " + Subsystems.intake.isNoteDetected());
                            return Subsystems.intake.isNoteDetected();
                        }
                ),
//
//                Commands.runOnce(() -> BSLogger.log("DropShot3", "@@@@@@@ RUNNING DROP SHOT 3")),
//                this.runAutoPath("DropShot3"),
//                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.FeedNoteToShooter),
//                DoShotCommand(shotProfile),
//                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Pickup),

                new RotateToAngle(90),
                Commands.runOnce(() -> BSLogger.log("DropShot4", "@@@@@@@ RUNNING DROP SHOT 4")),
                this.runAutoPath("DropShot4"),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.FeedNoteToShooter),
                DoShotCommand(shotProfile));
    }



    public Command ds2RouteCmd() {
        return Commands.sequence(
                Commands.runOnce(() -> BSLogger.log("ds2RouteCmd", "!!!!! RUNNING DS2 ROUTE COMMAND !!!!!")),
                this.runAutoPath("DropShot2"),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.FeedNoteToShooter),
                DoShotCommand(shotProfile));
//                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Pickup));
    }


    static Command DoShotCommand(VisionAimManager.ShootingProfile profile) {
        return Commands.sequence(
                // Start point is that we expect the shooter to have a note
                new WaitShooterHasNote().withTimeout(1.5),
                Commands.parallel(
                        new EnableShooterCommand(),
                        Commands.runOnce(() -> Subsystems.intake.setIntakeState(Intake.IntakeState.IntakeFromFloor)),
                        Commands.runOnce(() -> Subsystems.shooter.applyShootingProfile(profile)),
                        Commands.runOnce(() -> Subsystems.pivot.applyShootingProfile(profile))
                ),
                new WaitPivotInPosition().withTimeout(0.5),
                Commands.runOnce(() -> BSLogger.log("DoShotCommand", "******* WILL BE FIRING NEXT *******")),
                new WaitCommand(0.5),
                fire()
        ).beforeStarting(() -> BSLogger.log("DoShotCommand", "starting"))
                .finallyDo(() -> BSLogger.log("DoShotCommand", "ending"));
    }

    static Command fire() {
        return Commands.parallel(
                Commands.runOnce(() -> BSLogger.log("doShootCmd", "shooting")),
                Commands.runOnce(Subsystems.shooter::shoot).andThen(new WaitCommand(0.2))
        );
    }


    public static void registerAutoPaths(PathRegistry pathRegistry) {
        pathRegistry.registerPaths("DropShot", "DropShot2", "DropShot3", "DropShot4");
    }
}
