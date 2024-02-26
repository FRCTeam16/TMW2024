package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.auto.EnableShooterCommand;
import frc.robot.commands.auto.InitializeAutoState;
import frc.robot.commands.auto.WaitShooterHasNote;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.VisionAimManager;
import frc.robot.subsystems.pose.PoseManager;

public class UnderTheBridge extends SequentialCommandGroup {
    public static final VisionAimManager.ShootingProfile SecondShotProfile = new VisionAimManager.ShootingProfile(23.19, 59.266, 38.799); // 129ish?
    public static final VisionAimManager.ShootingProfile ThirdShotProfile = new VisionAimManager.ShootingProfile(45.85, 18.27, 20.965); // 55"

    public UnderTheBridge() {
        addCommands(
                Commands.parallel(
                        new PrintCommand("Under the Bridge"),
                        new InitializeAutoState(0),
                        new EnableShooterCommand()
                ),
                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.ShortShot),
                new WaitCommand(0.5),
                Commands.runOnce(Subsystems.shooter::shoot),
                new WaitCommand(0.5),
                Commands.parallel(
                        new EnableShooterCommand(false),
                        Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Drive)
                ),
                Subsystems.swerveSubsystem.getAutoPath("UnderTheBridge"),
                Subsystems.pivot.moveToPositionCmd(Pivot.PivotPosition.Custom),
                new WaitCommand(0.25),
                Commands.runOnce(Subsystems.shooter::shoot),
                new WaitCommand(0.5),
                new EnableShooterCommand(false),

                //
                // UTB 2
                //
                Subsystems.swerveSubsystem.getAutoPath("UnderTheBridge2"),
                Commands.parallel(
                        Subsystems.poseManager.getPoseCommand(PoseManager.Pose.FeedNoteToShooter),
                        new WaitShooterHasNote()),
                Commands.runOnce(Subsystems.shooter::shoot).andThen(new WaitCommand(0.5))
        );

    }
}
