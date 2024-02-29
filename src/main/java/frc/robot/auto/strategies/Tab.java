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

    public static final VisionAimManager.ShootingProfile SecondShot = new VisionAimManager.ShootingProfile(38, 30, 26); // 87"
    public static final VisionAimManager.ShootingProfile ThirdShot = new VisionAimManager.ShootingProfile(43, 22, 22); // 55"
    public static final VisionAimManager.ShootingProfile ForthShot = new VisionAimManager.ShootingProfile(37, 31, 27); // 76.5"
    public static final VisionAimManager.ShootingProfile FifthShot = new VisionAimManager.ShootingProfile(26.5, 45, 32); // 98.8"


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
                Commands.parallel(
                        Subsystems.poseManager.getPoseCommand(PoseManager.Pose.ShortShot),
                        new WaitCommand(0.25)
                ),
                doShootCmd(),
                Commands.parallel(
                        new EnableShooterCommand(false),
                        Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Drive)
                ),

                //
                // Run and pickup, second shot
                //
                Commands.parallel(
                        Commands.runOnce(() -> BSLogger.log("Tab", "****** Running Tab")),
                        Subsystems.pivot.setQueuedProfileCmd(SecondShot)
                ),
                this.runAutoPath("Tab"),
                Commands.runOnce(() -> Subsystems.pivot.applyShootingProfile(SecondShot)),
                new RotateToAngle(-32).withTimeout(0.5),
                DoShotCommand(SecondShot),


                //
                // Pickup, third shot
                //
                Commands.parallel(
                        Commands.runOnce(() -> BSLogger.log("Tab", "****** Running Tab2")),
                        Subsystems.pivot.setQueuedProfileCmd(ThirdShot)
                ),
                this.runAutoPath("Tab2"),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.FeedNoteToShooter),  // trying this to make sure we are in a good state, try to prevent feed through
                DoShotCommand(ThirdShot),

                //
                // Pickup, forth shot
                //
                Commands.parallel(
                        Commands.runOnce(() -> BSLogger.log("Tab", "****** Running Tab3")),
                        Subsystems.pivot.setQueuedProfileCmd(ForthShot)
                ),
                this.runAutoPath("Tab3"),
                // May need to check state here, auto command is dropping out
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.FeedNoteToShooter),
                DoShotCommand(ForthShot),

                //
                // Pickup, fifth shot
                //
                Commands.parallel(
                        Commands.runOnce(() -> BSLogger.log("Tab", "****** Running Tab4")),
                        Subsystems.pivot.setQueuedProfileCmd(FifthShot)
                ),
                this.runAutoPath("Tab4"),
                Commands.parallel(
                        Commands.runOnce(() -> BSLogger.log("FeedNoteInAuto", "starting")),
                        new WaitIntakeHasNoteCommand(),
                        new WaitIntakeInPosition(IntakePivot.IntakePosition.Zero)
                ).withTimeout(3.0),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.FeedNoteToShooter),
                DoShotCommand(FifthShot)
        );
    }

    static Command DoShotCommand(VisionAimManager.ShootingProfile profile) {
        return new SequentialCommandGroup(
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
                doShootCmd(),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Pickup)
        ).beforeStarting(() -> BSLogger.log("DoShotCommand", "starting"))
                .finallyDo(() -> BSLogger.log("DoShotCommand", "ending"));
    }


    public static void registerAutoPaths(PathRegistry pathRegistry) {
        pathRegistry.registerPaths("Tab", "Tab2", "Tab3", "Tab4");
    }

    static Command doShootCmd() {
        return Commands.parallel(
                Commands.runOnce(() -> BSLogger.log("doShootCmd", "shooting")),
                Commands.runOnce(Subsystems.shooter::shoot).andThen(new WaitCommand(0.2))
        );
    }
}
