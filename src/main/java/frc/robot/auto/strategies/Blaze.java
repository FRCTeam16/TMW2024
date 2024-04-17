package frc.robot.auto.strategies;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.subsystems.trap.Trap;
import frc.robot.subsystems.util.GameInfo;

import static edu.wpi.first.units.Units.Degrees;

public class Blaze extends AutoPathStrategy {
    private final Pose2d blueStartPose = new Pose2d(1.4478, 2.184, new Rotation2d(Degrees.of(-90)));
    private final VisionAimManager.ShootingProfile firstShotProfile = new VisionAimManager.ShootingProfile(20.74, 52.5, 52);
    public static final VisionAimManager.ShootingProfile shootingPositionProfile = new VisionAimManager.ShootingProfile(17, 56, 56);

    public enum BlazeRoute {
        BlazeAB(new RouteNames("BlazeAB", "BlazeAB2", "BlazeAB2a")),
        BlazeBA(new RouteNames("BlazeBA", "BlazeBA2", "BlazeBA2a")),
        BlazeBC(new RouteNames("BlazeBC", "BlazeBC2", "BlazeBC2a")),
        BlazeCB(new RouteNames("BlazeCB", "BlazeCB2", "BlazeCB2a")),
        BlazeAC(new RouteNames("BlazeAC", "BlazeAC2", "BlazeAC2a"));
//        BlazeCA;

        private final RouteNames routeNames;

        BlazeRoute(RouteNames routeNames) {
            this.routeNames = routeNames;
        }
    }

    public record RouteNames(String base, String next, String alternate) {}


    public Blaze(BlazeRoute route) {
        addCommands(
                Commands.parallel(
                        new PrintStartInfo("Blaze route: " + route),
                        new InitializeAutoState(Degrees.of(0)),
                        new EnableShooterCommand(),
                        Subsystems.trap.moveToStateCmd(Trap.TrapState.Drive)
                ),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Drive),

                //
                // Configure starting pose
                //
                Commands.runOnce(
                        () -> {
                            if (GameInfo.isBlueAlliance()) {
                                Subsystems.swerveSubsystem.seedFieldRelative(blueStartPose);
                            } else {
                                Subsystems.swerveSubsystem.seedFieldRelative(
                                        GeometryUtil.flipFieldPose(blueStartPose));
                            }
                        }
                ),

                //
                // First Shot
                //
                Commands.parallel(
                        new RotateToAngle(-69)
                                .withThreshold(1).withTimeout(0.5),
                        Commands.runOnce(() -> Subsystems.shooter.applyShootingProfile(firstShotProfile)),
                        Commands.runOnce(() -> Subsystems.pivot.applyShootingProfile(firstShotProfile)),
                        new WaitCommand(0.25)
                ).withTimeout(0.5),
                CommonCommands.doShootCmd(),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Drive),

                //
                // First leg to pickup
                //
                writeLog(route.name(), "First leg to pickup"),
                this.runAutoPath(route.routeNames.base),

                //
                // Decision Point
                //
                Commands.either(
                        this.runAutoPath(route.routeNames.next),
                        this.runAutoPath(route.routeNames.alternate),
                        Subsystems.intake::isNoteDetected
                )
        ); // end addCommands
    }


    public static void registerAutoPaths(PathRegistry pathRegistry) {
        for(BlazeRoute route : BlazeRoute.values()) {
            pathRegistry.registerPaths(
                    route.routeNames.base,
                    route.routeNames.next,
                    route.routeNames.alternate);
        }
    }
}
