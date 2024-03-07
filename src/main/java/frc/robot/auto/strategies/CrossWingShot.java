package frc.robot.auto.strategies;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.auto.PathRegistry;
import frc.robot.commands.auto.EnableShooterCommand;
import frc.robot.commands.auto.InitializeAutoState;
import frc.robot.commands.auto.PrintStartInfo;
import frc.robot.commands.auto.WaitShooterHasNote;
import frc.robot.subsystems.VisionAimManager;
import frc.robot.subsystems.pose.PoseManager;

public class CrossWingShot extends AutoPathStrategy {
    //TODO: GET NUMBERS
    public static final VisionAimManager.ShootingProfile PointBlankShot = new VisionAimManager.ShootingProfile(23.19, 59.266, 38.799);
    public static final VisionAimManager.ShootingProfile WingShot = new VisionAimManager.ShootingProfile(23.19, 59.266, 38.799); // 129ish?
    

    public static void registerAutoPaths(PathRegistry registry) {
        registry.registerPaths("StartToNCL4", "NCL4ToWing", "WingToNCL5", "NCL5ToWing", "WingToNCL3", "NCL3ToWing");
    }

    public CrossWingShot(){
        addCommands( //Standard - just like last year
            Commands.parallel(
                        new PrintStartInfo("Crosswing Shot"),
                        new InitializeAutoState(Degrees.of(0)), 
                        new EnableShooterCommand()
//                        Subsystems.swerveSubsystem.applyRequest(() -> pointWheels)
            ),

            Subsystems.poseManager.getPoseCommand(PoseManager.Pose.ShortShot),
                new WaitCommand(0.25),
                Commands.runOnce(Subsystems.shooter::shoot).andThen(new WaitCommand(0.2)),
                Commands.parallel(
                        new EnableShooterCommand(false),
                        Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Drive)
                ),

                this.runAutoPath("StartToNCL4"),
                
                // NOW AT FIRST NOTE ( also working on triggering pose switch while driving )



                this.runAutoPath("NCL4ToWing"),
                // NOW AT WING SHOT ( 1 )
                    Commands.parallel(
                    Commands.runOnce(() -> Subsystems.pivot.applyShootingProfile(WingShot)),
                    new EnableShooterCommand()
                ),
                new WaitCommand(0.5),
                Commands.runOnce(Subsystems.shooter::shoot).andThen(new WaitCommand(0.2)),
                
                this.runAutoPath("WingToNCL5"),
                this.runAutoPath("NCL5ToWing"), // WING SHOT ( 2 )
        
                    Commands.parallel(
                    Commands.runOnce(() -> Subsystems.pivot.applyShootingProfile(WingShot)),
                    new EnableShooterCommand()
                ),

            this.runAutoPath("WingToNCL3"),
            this.runAutoPath("NCL3ToWing"),
                
                Commands.parallel(
                    Commands.runOnce(() -> Subsystems.pivot.applyShootingProfile(WingShot)),
                    new EnableShooterCommand()
                )

        );
    }
}
