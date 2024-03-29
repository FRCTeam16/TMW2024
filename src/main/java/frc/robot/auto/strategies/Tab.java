package frc.robot.auto.strategies;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.auto.PathRegistry;
import frc.robot.commands.auto.EnableShooterCommand;
import frc.robot.commands.auto.InitializeAutoState;
import frc.robot.commands.auto.PrintStartInfo;
import frc.robot.commands.auto.RotateToAngle;
import frc.robot.subsystems.VisionAimManager;
import frc.robot.subsystems.pose.PoseManager;
import frc.robot.subsystems.util.GameInfo;

import static edu.wpi.first.units.Units.Degrees;

public class Tab extends AutoPathStrategy {

    public static final VisionAimManager.ShootingProfile TabStraightFirst = new VisionAimManager.ShootingProfile(41.5, 30, 26); // 87"
    public static final VisionAimManager.ShootingProfile SecondShot = new VisionAimManager.ShootingProfile(40.0, 30, 26); // 87"
    public static final VisionAimManager.ShootingProfile ThirdShot = new VisionAimManager.ShootingProfile(51.5, 22, 22); // 55"
    public static final VisionAimManager.ShootingProfile ForthShot = new VisionAimManager.ShootingProfile(38.5, 31, 27); // 76.5"
    public static final VisionAimManager.ShootingProfile FifthShot = new VisionAimManager.ShootingProfile(31.5, 45, 32); // 98.8"
    private final Pose2d blueStraightStartPose = new Pose2d(1.28, 4.79, Rotation2d.fromDegrees(0));

    public Tab(TabVersion version) {
        if (TabVersion.OffsetStart == version) {
            createTabOffsetStartCommands();
        } else {
            createTabStraightCommands();
        }
    }

    public static void registerAutoPaths(PathRegistry pathRegistry) {
        pathRegistry.registerPaths("XTab", "XTab1a");
    }

    static Command doShootCmd() {
        return Commands.parallel(
                writeLog("doShootCmd", "shooting"),
                Subsystems.shooter.shootCmd()
        );
    }

    public static Command createTabShot2Rotate() {
        return Commands.either(
                new RotateToAngle(30).withThreshold(5),
                new RotateToAngle(-30).withThreshold(5),
                GameInfo::isBlueAlliance).withTimeout(0.5);
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
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Drive),

                //
                // Run and pickup, second shot
                //
                Commands.parallel(
                        writeLog("Tab", "****** Running XTab"),
                        Subsystems.pivot.setQueuedProfileCmd(SecondShot)
                ),
                this.runAutoPath("XTab")
        );
    }

    void createTabStraightCommands() {
        addCommands(
                Commands.parallel(
                        new PrintStartInfo("Tab Straight"),
                        new InitializeAutoState(Degrees.of(0)),
                        new EnableShooterCommand()
                ),
                Commands.runOnce(
                        () -> {
                            if (GameInfo.isBlueAlliance()) {
                                Subsystems.swerveSubsystem.seedFieldRelative(blueStraightStartPose);
                            } else {
                                Subsystems.swerveSubsystem.seedFieldRelative(
                                        GeometryUtil.flipFieldPose(blueStraightStartPose));
                            }

                        }
                ),
                new RotateToAngle(
                        GameInfo.isBlueAlliance() ? 18 : GeometryUtil.flipFieldRotation(Rotation2d.fromDegrees(18)).getDegrees())
                        .withThreshold(3)
                        .withTimeout(0.5),

                //
                // First Shot
                //
                writeLog("Tab", "First Shot"),
                // TODO: Do we need to use tabFirstShotProfile
                Commands.parallel(
                        Subsystems.poseManager.getPoseCommand(PoseManager.Pose.AutoShortShot),
                        new WaitCommand(0.25)
                ),
                doShootCmd(),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Drive),

                //
                // Run and pickup, second shot
                //
                Commands.parallel(
                        writeLog("Tab", "****** Running XTab1a"),
                        Subsystems.pivot.setQueuedProfileCmd(SecondShot)
                ),
                this.runAutoPath("XTab1a")
        );
    }

    public enum TabVersion {
        StraightStart,
        OffsetStart
    }
}
