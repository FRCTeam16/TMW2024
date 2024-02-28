package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.auto.PathRegistry;
import frc.robot.commands.auto.*;
import frc.robot.subsystems.VisionAimManager;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.pose.PoseManager;
import frc.robot.subsystems.util.BSLogger;

public class Tab extends AutoPathStrategy {

    public static final VisionAimManager.ShootingProfile SecondShot = new VisionAimManager.ShootingProfile(37, 30, 26); // 87"
    public static final VisionAimManager.ShootingProfile ThirdShot = new VisionAimManager.ShootingProfile(42,22, 22); // 55"
    public static final VisionAimManager.ShootingProfile ForthShot = new VisionAimManager.ShootingProfile(36, 31, 27); // 76.5"
    public static final VisionAimManager.ShootingProfile FifthShot = new VisionAimManager.ShootingProfile(28.5, 45, 32); // 98.8"




    public Tab() {
        addCommands(
                Commands.parallel(
                        new PrintStartInfo("Tab"),
                        new InitializeAutoState(0),
                        new EnableShooterCommand()
//                        Subsystems.swerveSubsystem.applyRequest(() -> pointWheels)
                ),

                //
                // First Shot
                //
                Commands.runOnce(() -> BSLogger.log("Tab", "First Shot")),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.ShortShot),
                new WaitCommand(0.25),
                Commands.runOnce(Subsystems.shooter::shoot).andThen(new WaitCommand(0.2)),
                Commands.parallel(
                        new EnableShooterCommand(false),
                        Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Drive)
                ),

                //
                // Run and pickup, second shot
                //
                Commands.runOnce(() -> BSLogger.log("Tab", "Running Tab")),
                this.runAutoPath("Tab"),
                DoShotCommand(SecondShot),

                //
                // Pickup, third shot
                //
                Commands.runOnce(() -> BSLogger.log("Tab", "Running Tab2")),
                this.runAutoPath("Tab2"),
                DoShotCommand(ThirdShot),

                //
                // Pickup, forth shot
                //
                Commands.runOnce(() -> BSLogger.log("Tab", "Running Tab3")),
                this.runAutoPath("Tab3"),
                DoShotCommand(ForthShot),

                //
                // Pickup, fifth shot
                //
                Commands.runOnce(() -> BSLogger.log("Tab", "Running Tab4")),
                this.runAutoPath("Tab4"),
                Commands.parallel(
                    new WaitIntakeHasNoteCommand().withTimeout(1.0),
                    new WaitIntakeInPosition(IntakePivot.IntakePosition.Zero).withTimeout(1.0)
                ),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.FeedNoteToShooter),
                DoShotCommand(FifthShot)
        );
    }

    static Command DoShotCommand(VisionAimManager.ShootingProfile profile) {
        return new SequentialCommandGroup(
                // Start point is waiting for feed action to be completed
                new WaitShooterHasNote().withTimeout(1.5),
                new EnableShooterCommand(),
                Commands.parallel(
                        Commands.runOnce(() -> Subsystems.intake.setIntakeState(Intake.IntakeState.IntakeFromFloor)),
                        Commands.runOnce(() -> Subsystems.pivot.applyShootingProfile(profile))
                ),
                new WaitPivotInPosition().withTimeout(0.5),
                new WaitCommand(0.5),
                Commands.runOnce(Subsystems.shooter::shoot).andThen(new WaitCommand(0.2)),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Pickup)
        ).beforeStarting(() -> BSLogger.log("DoShotCommand", "starting"));
    }

    public static void registerAutoPaths(PathRegistry pathRegistry) {
        pathRegistry.registerPaths("Tab", "Tab2", "Tab3", "Tab4");
    }
}
