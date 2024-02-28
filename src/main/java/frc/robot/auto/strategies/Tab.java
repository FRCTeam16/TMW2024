package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.auto.PathRegistry;
import frc.robot.commands.auto.*;
import frc.robot.subsystems.VisionAimManager;
import frc.robot.subsystems.pose.PoseManager;

public class Tab extends AutoPathStrategy {

    public static final VisionAimManager.ShootingProfile SecondShot = new VisionAimManager.ShootingProfile(34.09, 35.998, 28.677); // 87"
    public static final VisionAimManager.ShootingProfile ThirdShot = new VisionAimManager.ShootingProfile(44.35, 18.27, 20.965); // 55"
    public static final VisionAimManager.ShootingProfile ForthShot = new VisionAimManager.ShootingProfile(37.783, 30.181, 26.1465); // 76.5"
    public static final VisionAimManager.ShootingProfile FifthShot = new VisionAimManager.ShootingProfile(29.528, 42.5252, 31.52); // 98.8"




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
                this.runAutoPath("Tab"),
                DoShotCommand(SecondShot),

                //
                // Pickup, third shot
                //
                this.runAutoPath("Tab2"),
                DoShotCommand(ThirdShot),

                //
                // Pickup, forth shot
                //
                this.runAutoPath("Tab3"),
                DoShotCommand(ForthShot),

                //
                // Pickup, fifth shot
                //
                this.runAutoPath("Tab4"),
                DoShotCommand(FifthShot)
        );
    }

    static Command DoShotCommand(VisionAimManager.ShootingProfile profile) {
        return new SequentialCommandGroup(
                // Start point is waiting for feed action to be completed
                new WaitShooterHasNote().withTimeout(1.5),
                Commands.parallel(
                        Commands.runOnce(() -> Subsystems.pivot.applyShootingProfile(profile)),
                        new EnableShooterCommand()
                ),
                new WaitCommand(0.5),
                Commands.runOnce(Subsystems.shooter::shoot).andThen(new WaitCommand(0.2)),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Pickup)
        );
    }

    public static void registerAutoPaths(PathRegistry pathRegistry) {
        pathRegistry.registerPaths("Tab");
    }
}
