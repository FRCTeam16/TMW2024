package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.auto.PathRegistry;
import frc.robot.commands.auto.EnableShooterCommand;
import frc.robot.commands.auto.InitializeAutoState;
import frc.robot.subsystems.pose.PoseManager;

import static edu.wpi.first.units.Units.Degrees;

public class DebugAuto extends SequentialCommandGroup {
  public DebugAuto() {
    // double speed = 0.5;
    // double offset = -90.0;
    addCommands(
            new InitializeAutoState(Degrees.of(0)),
            new PrintCommand("DEBUG AUTO"),
            Subsystems.poseManager.getPoseCommand(PoseManager.Pose.ShortShot),
            new WaitCommand(2.0),
            Commands.runOnce(Subsystems.shooter::shoot),
            new WaitCommand(1.0),
            new EnableShooterCommand(false),
            new PrintCommand("Shooter Enabled finsihed"),
            new WaitCommand(0.5),
            Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Pickup),
            new WaitCommand(1.0),
            Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Drive)
            // new RotateToAngle(0)
            //   .withTimeout(3)
    );
  }

}
