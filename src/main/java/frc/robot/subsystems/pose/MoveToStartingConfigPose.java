package frc.robot.subsystems.pose;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class MoveToStartingConfigPose extends SequentialCommandGroup {
    public MoveToStartingConfigPose() {
        addCommands(
                Commands.parallel(
//                        Subsystems.intake.setPosition(Intake.IntakePosition.StartingPosition),
//                        Subsystems.shooter.setPosition(Shooter.ShooterPosition.StartingPosition)
                )
        );

    }
}
