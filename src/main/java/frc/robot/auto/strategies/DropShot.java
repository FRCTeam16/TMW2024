package frc.robot.auto.strategies;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.proto.Geometry2D;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.subsystems.util.GameInfo;

import static edu.wpi.first.units.Units.Degree;

public class DropShot extends AutoPathStrategy {

    private final VisionAimManager.ShootingProfile firstShotProfile = new VisionAimManager.ShootingProfile(36.46, 40, 40);
    private final VisionAimManager.ShootingProfile fieldShotProfile = new VisionAimManager.ShootingProfile(20.3, 55, 55);


    public DropShot() {
        addCommands(

                //
                // Initialize the robot
                //
                Commands.parallel(
                        new PrintStartInfo("DropShot"),
                        new InitializeAutoState(Degree.of(0)),
                        new EnableShooterCommand(true)
//                        pointWheelsAtCmd(Degree.of(0))
                ),
                Commands.runOnce(
                        () -> {
                            if (GameInfo.isBlueAlliance()) {
                                Subsystems.swerveSubsystem.seedFieldRelative(
                                        new Pose2d(1.84, 7.44, new Rotation2d(Degree.of(0))));
                            } else {
                                Subsystems.swerveSubsystem.seedFieldRelative(
                                        GeometryUtil.flipFieldPose( // Flip the field pose for red alliance
                                                new Pose2d(1.84, 7.44, new Rotation2d(Degree.of(0)))));
                            }
                        }

                ),
//                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.PrepareBloopShot),
                Commands.parallel(
                        new RotateToAngle(50).withThreshold(5).withTimeout(0.5),
                        Commands.runOnce(() -> Subsystems.shooter.applyShootingProfile(firstShotProfile)),
                        Commands.runOnce(() -> Subsystems.pivot.applyShootingProfile(firstShotProfile)),
                        new WaitCommand(0.25)
                ).withTimeout(0.5),
                Tab.doShootCmd(),
                new RotateToAngle(0).withThreshold(5).withTimeout(0.5),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Drive),

                //
                // Run first leg and pickup
                //
                writeLog("DropShot", "Running DropShot1"),
                this.runAutoPath("DropShot"),

                //
                // Select which route to run based on whether we have a note or not
                //
                writeLog("DropShot", () -> "********************** NOTE DETECTED: " + Subsystems.intake.isNoteDetected()),
                Commands.either(
//                        RouteCommand("DropShot2").andThen(RouteCommand("DropShot3a")),
//                        Commands.sequence(
//                                RouteCommand("DropShot2"),
//                                RouteCommand("DropShot3a")
//                        ),
                        RouteCommand("DropShot2"),
                        RouteCommand("DropShot3"),
                        () -> Subsystems.intake.isNoteDetected()
                ),

                //
                // Fourth route runs and gets the note we dropped
                //
                writeLog("DropShot", "Running DropShot4"),
                this.runAutoPath("DropShot4"),
                writeLog("DropShot", "Finished DropShot4"),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.FeedNoteToShooter),
                Tab.DoShotCommand(fieldShotProfile),
                Commands.print("@@@@ Finished DropShot @@@@"));
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
                writeLog("DropShot3", "Finished drive portion of : " + pathName),
                Commands.runOnce(() -> Subsystems.shooter.applyShootingProfile(fieldShotProfile)),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.FeedNoteToShooter),
                Commands.runOnce(() -> Subsystems.pivot.applyShootingProfile(fieldShotProfile)),
                new WaitCommand(0.25),
                Tab.DoShotCommand(fieldShotProfile),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Pickup),
                writeLog("DropShot", "!!!!! FINISHED ROUTE COMMAND: " + pathName)
        );
    }

    Command DoShotCommand(VisionAimManager.ShootingProfile profile) {
        return Commands.sequence(
                        // Start point is that we expect the shooter to have a note
                        new WaitShooterHasNote().withTimeout(0.5),
                        Commands.runOnce(() -> Subsystems.intake.setIntakeState(Intake.IntakeState.IntakeFromFloor)),
                        Commands.sequence(
                                Commands.parallel(
                                        new EnableShooterCommand(),
                                        Commands.runOnce(() -> Subsystems.shooter.applyShootingProfile(profile)),
                                        Commands.runOnce(() -> Subsystems.pivot.applyShootingProfile(profile))
                                )

                        ),
                        new WaitPivotInPosition().withTimeout(0.5),
//                        new WaitCommand(0.5),
                        fire()
                ).beforeStarting(() -> BSLogger.log("DoShotCommand", "starting"))
                .finallyDo(() -> BSLogger.log("DoShotCommand", "ending"));
    }
}
