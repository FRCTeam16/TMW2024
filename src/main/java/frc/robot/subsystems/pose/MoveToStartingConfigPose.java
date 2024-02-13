package frc.robot.subsystems.pose;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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
