package frc.robot.auto.strategies;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Subsystems;
import frc.robot.auto.PathRegistry;
import frc.robot.commands.auto.*;
import frc.robot.subsystems.VisionAimManager;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.pose.PoseManager;

import static edu.wpi.first.units.Units.Degrees;

public class UnderTheBridge extends AutoPathStrategy {
    public static final VisionAimManager.ShootingProfile SecondShotProfile = new VisionAimManager.ShootingProfile(23.19, 59.266, 38.799); // 129ish?
    public static final VisionAimManager.ShootingProfile ThirdShotProfile = new VisionAimManager.ShootingProfile(45.85, 18.27, 20.965); // 55"
    public static final VisionAimManager.ShootingProfile ForthShotProfile = new VisionAimManager.ShootingProfile(41, 18.27, 20.965); // 55"
    public static final VisionAimManager.ShootingProfile FifthShotProfile = new VisionAimManager.ShootingProfile(33.09, 35.998, 28.677); // 87"


    private SwerveRequest.PointWheelsAt pointWheels = new SwerveRequest.PointWheelsAt().withModuleDirection(Rotation2d.fromDegrees(0));


    public static void registerAutoPaths(PathRegistry registry) {
        registry.registerPaths("UnderTheBridge", "UnderTheBridge2", "UnderTheBridge3", "UnderTheBridge4");
    }

    public UnderTheBridge() {
        addCommands(
                Commands.parallel(
                        new PrintStartInfo("Under the Bridge"),
                        new InitializeAutoState(Degrees.of(0)),
                        new EnableShooterCommand()
//                        Subsystems.swerveSubsystem.applyRequest(() -> pointWheels)
                ),

                //
                // First Shot
                //
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.AutoShortShot),
                new WaitCommand(0.25),
                Commands.runOnce(Subsystems.shooter::shoot).andThen(new WaitCommand(0.2)),
                Commands.parallel(
                        new EnableShooterCommand(false),
                        Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Drive)
                ),

                //
                // UTB and pickup from line
                //
                this.runAutoPath("UnderTheBridge"),

                //
                // Do Second Shot
                //
//                handled in drive
//                Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.Custom),
//                new WaitCommand(0.25),
//                Commands.runOnce(Subsystems.shooter::shoot).andThen(new WaitCommand(0.2)),
                new EnableShooterCommand(false),

                //
                // UndertheBridge 2
                //
                this.runAutoPath("UnderTheBridge2"),

                //
                // Third shot
                //
                new WaitShooterHasNote().withTimeout(1.0),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Pickup),
                    Commands.parallel(
                    Commands.runOnce(() -> Subsystems.pivot.applyShootingProfile(ThirdShotProfile)),
                    new EnableShooterCommand()
                ),
                new WaitCommand(0.5),
                Commands.runOnce(Subsystems.shooter::shoot).andThen(new WaitCommand(0.2)),

                ///
                // UTB 3
                //
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Pickup),
                this.runAutoPath("UnderTheBridge3"),

                //
                // Fourth Shot
                //
                new WaitIntakeInPosition(IntakePivot.IntakePosition.Zero).withTimeout(1.0),
                new WaitShooterHasNote().withTimeout(1.0),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Pickup),
                Commands.parallel(
                        Commands.runOnce(() -> Subsystems.pivot.applyShootingProfile(ForthShotProfile)),
                        new EnableShooterCommand()
                ),
                new WaitCommand(0.5),
                Commands.runOnce(Subsystems.shooter::shoot).andThen(new WaitCommand(0.2)),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Pickup),


                ///
                // UTB 4
                //
                this.runAutoPath("UnderTheBridge4"),


                //
                // Fifth Shot
                //
                new WaitIntakeInPosition(IntakePivot.IntakePosition.Zero).withTimeout(1.0),
                new WaitIntakeHasNoteCommand().withTimeout(1.0),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.FeedNoteToShooter),
                new WaitShooterHasNote().withTimeout(1.0),
                Commands.parallel(
                        Commands.runOnce(() -> Subsystems.pivot.applyShootingProfile(FifthShotProfile)),
                        new EnableShooterCommand()
                ),
                new WaitCommand(0.5),
                Commands.runOnce(Subsystems.shooter::shoot).andThen(new WaitCommand(0.2))
        );
    }

    public Command waitForPositionThenShoot() {
        return new SequentialCommandGroup(

        );
    }
}
