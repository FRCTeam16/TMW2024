package frc.robot.auto.strategies;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Subsystems;
import frc.robot.commands.auto.*;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.VisionAimManager;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.pose.PoseManager;

public class UnderTheBridge extends SequentialCommandGroup {
    public static final VisionAimManager.ShootingProfile SecondShotProfile = new VisionAimManager.ShootingProfile(23.19, 59.266, 38.799); // 129ish?
    public static final VisionAimManager.ShootingProfile ThirdShotProfile = new VisionAimManager.ShootingProfile(45.85, 18.27, 20.965); // 55"
    public static final VisionAimManager.ShootingProfile ForthShotProfile = new VisionAimManager.ShootingProfile(41, 18.27, 20.965); // 55"
    public static final VisionAimManager.ShootingProfile FifthShotProfile = new VisionAimManager.ShootingProfile(33.09, 35.998, 28.677); // 87"


    private SwerveRequest.PointWheelsAt pointWheels = new SwerveRequest.PointWheelsAt().withModuleDirection(Rotation2d.fromDegrees(0));
    public UnderTheBridge() {
        addCommands(
                Commands.parallel(
                        new PrintStartInfo("Under the Bridge"),
                        new PrintCommand("Under the Bridge"),
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
                // UTB and pickup from line
                //
                Subsystems.swerveSubsystem.getAutoPath("UnderTheBridge"),

                //
                // Do Second Shot
                //
                Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.Custom),
                new WaitCommand(0.25),
                Commands.runOnce(Subsystems.shooter::shoot).andThen(new WaitCommand(0.2)),
                new EnableShooterCommand(false),

                //
                // UndertheBridge 2
                //
                Subsystems.swerveSubsystem.getAutoPath("UnderTheBridge2"),

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
                Subsystems.swerveSubsystem.getAutoPath("UnderTheBridge3"),

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
                Subsystems.swerveSubsystem.getAutoPath("UnderTheBridge4"),


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
