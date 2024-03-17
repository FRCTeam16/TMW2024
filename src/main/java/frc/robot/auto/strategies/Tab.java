package frc.robot.auto.strategies;

import com.pathplanner.lib.commands.PathPlannerAuto;
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

import static edu.wpi.first.units.Units.Degrees;

public class Tab extends AutoPathStrategy {

    public static final VisionAimManager.ShootingProfile TabStraightFirst = new VisionAimManager.ShootingProfile(41.5, 30, 26); // 87"
    public static final VisionAimManager.ShootingProfile SecondShot = new VisionAimManager.ShootingProfile(41.5, 30, 26); // 87"
    public static final VisionAimManager.ShootingProfile ThirdShot = new VisionAimManager.ShootingProfile(47.5, 22, 22); // 55"
    public static final VisionAimManager.ShootingProfile ForthShot = new VisionAimManager.ShootingProfile(43.5, 31, 27); // 76.5"
    public static final VisionAimManager.ShootingProfile FifthShot = new VisionAimManager.ShootingProfile(31.5, 45, 32); // 98.8"
    private PathPlannerAuto tab1 = new PathPlannerAuto("Tab");
    private PathPlannerAuto tab1a = new PathPlannerAuto("Tab1a");
    private PathPlannerAuto tab2 = new PathPlannerAuto("Tab2");
    private PathPlannerAuto tab3 = new PathPlannerAuto("Tab3");
    private PathPlannerAuto tab4 = new PathPlannerAuto("Tab4");


    public Tab(TabVersion version) {
        if (TabVersion.OffsetStart == version) {
            createTabOffsetStartCommands();
        } else {
            createTabStraightCommands();
        }
        createCommands();
    }

    static Command DoShotCommand(VisionAimManager.ShootingProfile profile) {
        return new SequentialCommandGroup(
                // Start point is that we expect the shooter to have a note
                new WaitShooterHasNote().withTimeout(0.5),
                Subsystems.intake.moveToStateCmd(Intake.IntakeState.MoveIntakeToFloorWithoutIntaking),
                writeLog("DoShotCommand", "If shooter has note then we will try shot"),
                //
                Commands.sequence(
                        Commands.parallel(
                                writeLog("DoShotCommand", "Setting shooter and pivot profiles"),
                                new EnableShooterCommand(),
                                Commands.runOnce(() -> Subsystems.shooter.applyShootingProfile(profile)),
                                Commands.runOnce(() -> Subsystems.pivot.applyShootingProfile(profile))
                        ),
                        new WaitPivotInPosition().withTimeout(0.5),
                        writeLog("DoShotCommand", "******* WILL BE FIRING NEXT *******"),
//                        new WaitCommand(0.5),
                        doShootCmd()
                ).unless(() -> !Subsystems.shooter.isNoteDetected()),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Pickup)
        ).beforeStarting(() -> BSLogger.log("DoShotCommand", "starting"))
                .finallyDo(() -> BSLogger.log("DoShotCommand", "ending"));
    }

    public static void registerAutoPaths(PathRegistry pathRegistry) {
//        pathRegistry.registerPaths("Tab", "Tab1a", "Tab2", "Tab3", "Tab4");
    }

    static Command doShootCmd() {
        return Commands.parallel(
                writeLog("doShootCmd", "shooting"),
                Subsystems.shooter.shootCmd()
        );
    }


    void createTabOffsetStartCommands() {
        addCommands(
                Commands.parallel(
                        new PrintStartInfo("Tab Offset"),
                        new InitializeAutoState(Degrees.of(0)),
                        new EnableShooterCommand()
                ),

                //
                // First Shot
                //
                writeLog("Tab", "First Shot"),
                Commands.parallel(
                        Subsystems.poseManager.getPoseCommand(PoseManager.Pose.AutoShortShot),
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
                        writeLog("Tab", "****** Running Tab"),
                        Subsystems.pivot.setQueuedProfileCmd(SecondShot)
                ),
                tab1
        );
    }

    void createTabStraightCommands() {
        addCommands(
                Commands.parallel(
                        new PrintStartInfo("Tab Straight"),
                        new InitializeAutoState(Degrees.of(0)),
                        new EnableShooterCommand(),
                        new RotateToAngle(-18).withThreshold(5).withTimeout(0.5)
                ),

                //
                // First Shot
                //
                writeLog("Tab", "First Shot"),
                Commands.parallel(
                        Subsystems.poseManager.getPoseCommand(PoseManager.Pose.AutoShortShot),
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
                        writeLog("Tab", "****** Running Tab1a"),
                        Subsystems.pivot.setQueuedProfileCmd(SecondShot)
                ),
                tab1a
        );
    }

    /**
     * Expected start staight is we are at second shot position
     */
    void createCommands() {
        addCommands(
                Commands.runOnce(() -> Subsystems.pivot.applyShootingProfile(SecondShot)),
                new RotateToAngle(-30).withThreshold(5).withTimeout(0.5),
                DoShotCommand(SecondShot),


                //
                // Pickup, third shot
                //
                Commands.parallel(
                        writeLog("Tab", "****** Running Tab2"),
                        Subsystems.pivot.setQueuedProfileCmd(ThirdShot)
                ),
                tab2,
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.FeedNoteToShooter),  // trying this to make sure we are in a good state, try to prevent feed through
                DoShotCommand(ThirdShot),

                //
                // Pickup, forth shot
                //
                Commands.parallel(
                        writeLog("Tab", "****** Running Tab3"),
                        Subsystems.pivot.setQueuedProfileCmd(ForthShot)
                ),
                tab3,
                // May need to check state here, auto command is dropping out
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.FeedNoteToShooter),
                DoShotCommand(ForthShot),

                //
                // Pickup, fifth shot
                //
                Commands.parallel(
                        writeLog("Tab", "****** Running Tab4"),
                        Subsystems.pivot.setQueuedProfileCmd(FifthShot)
                ),
                tab4,
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.FeedNoteToShooter),
                DoShotCommand(FifthShot),
                // Subsystems.poseManager.getPoseCommand(PoseManager.Pose.FeedNoteToShooter)
                writeLog("Tab auto", "@@@ FINISHED AUTO @@@")

        );
    }

    public enum TabVersion {
        StraightStart,
        OffsetStart
    }
}
