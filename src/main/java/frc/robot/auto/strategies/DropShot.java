package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.auto.PathRegistry;
import frc.robot.commands.auto.*;
import frc.robot.subsystems.VisionAimManager;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pose.PoseManager;
import frc.robot.subsystems.util.BSLogger;

import static edu.wpi.first.units.Units.Degree;

public class DropShot extends AutoPathStrategy {

    private final VisionAimManager.ShootingProfile shotProfile = new VisionAimManager.ShootingProfile(26.5, 45, 32);


    public DropShot() {
        addCommands(
                Commands.parallel(
                        new PrintStartInfo("DropShot"),
                        new InitializeAutoState(Degree.of(0)),
                        new EnableShooterCommand(true),
                        pointWheelsAtCmd(Degree.of(0))
                ),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.PrepareBloopShot),
                writeLog("DropShot", "Running"),
                this.runAutoPath("DropShot"),

                //
                // Select which route to run based on whether we have a note or not
                //
                writeLog("DropShot", () -> "********************** NOTE DETECTED: " + Subsystems.intake.isNoteDetected()),
                Commands.either(
                        RouteCommand("DropShot2").andThen(RouteCommand("DropShot3a")),
                        RouteCommand("DropShot3"),
                        () -> Subsystems.intake.isNoteDetected()
                ),

                //
                // Fourth route runs and gets the note we dropped
                //
                new RotateToAngle(90),
                writeLog("DropShot", "Running DropShot4"),
                this.runAutoPath("DropShot4"),
                writeLog("DropShot", "Finished DropShot4"),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.FeedNoteToShooter),
                DoShotCommand(shotProfile),
                Commands.print("Finished"));
    }

    public static void registerAutoPaths(PathRegistry pathRegistry) {
        pathRegistry.registerPaths("DropShot", "DropShot2", "DropShot3", "DropShot4");
    }


    static Command fire() {
        return Commands.parallel(
                writeLog("DropShot", "!!! FIRE !!!"),
                Commands.runOnce(Subsystems.shooter::shoot).andThen(new WaitCommand(0.2))
        );
    }


    /**
     * Command generator for the route commands that performs a feed then shot sequence, then ends with a pickup pose
     *
     * @param pathName the name of the path to run
     * @return the command to run the path, then feed the note to the shooter, then shoot, then pickup the note
     */
    Command RouteCommand(String pathName) {
        return Commands.sequence(
                writeLog("DropShot", "!!!!! RUNNING ROUTE COMMAND: " + pathName),
                this.runAutoPath(pathName),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.FeedNoteToShooter),
                DoShotCommand(shotProfile),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Pickup),
                writeLog("DropShot", "!!!!! FINISHED ROUTE COMMAND: " + pathName)
        );
    }

    Command DoShotCommand(VisionAimManager.ShootingProfile profile) {
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
                        new WaitCommand(0.5),
                        fire()
                ).beforeStarting(() -> BSLogger.log("DoShotCommand", "starting"))
                .finallyDo(() -> BSLogger.log("DoShotCommand", "ending"));
    }
}
