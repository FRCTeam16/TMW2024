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
    private final double startAngle = -90;
    private final double startingAngleOffset = 21; // 90 - 31
    private final Pose2d blueStartPose = new Pose2d(1.4478, 2.184, new Rotation2d(Degrees.of(-90)));
    private final VisionAimManager.ShootingProfile firstShotProfile = new VisionAimManager.ShootingProfile(20.74, 52.5, 52);
    public static final VisionAimManager.ShootingProfile shootingPositionProfile = new VisionAimManager.ShootingProfile(19, 56, 56);

    public Blaze() {
        addCommands(
                Commands.parallel(
                        new PrintStartInfo("Blaze"),
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
                        new RotateToAngle(GameInfo.isBlueAlliance() ? startAngle - startingAngleOffset : startAngle + startingAngleOffset)
                                .withThreshold(1).withTimeout(0.5),
                        Commands.runOnce(() -> Subsystems.shooter.applyShootingProfile(firstShotProfile)),
                        Commands.runOnce(() -> Subsystems.pivot.applyShootingProfile(firstShotProfile)),
                        new WaitCommand(0.25)
                ).withTimeout(0.5),
                Tab.doShootCmd(),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Drive),

                //
                // First leg to pickup
                //
                writeLog("Blaze", "First leg to pickup"),
                this.runAutoPath("Blaze"),

                //
                // Decision Point
                //
                Commands.either(
                        this.runAutoPath("Blaze2"),
                        this.runAutoPath("Blaze2a"),
                        Subsystems.intake::isNoteDetected
                )
        ); // end addCommands
    }


    public static void registerAutoPaths(PathRegistry pathRegistry) {
        pathRegistry.registerPaths("Blaze", "Blaze2", "Blaze2a");
    }
}
