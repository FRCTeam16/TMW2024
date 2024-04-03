package frc.robot.auto.strategies;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.auto.PathRegistry;
import frc.robot.commands.auto.EnableShooterCommand;
import frc.robot.commands.auto.InitializeAutoState;
import frc.robot.commands.auto.PrintStartInfo;
import frc.robot.commands.auto.WaitIntakeHasNoteCommand;
import frc.robot.commands.auto.WaitIntakeInPosition;
import frc.robot.commands.auto.WaitShooterHasNote;
import frc.robot.subsystems.VisionAimManager;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.pose.PoseManager;

import static edu.wpi.first.units.Units.Degrees;


public class SL1toSL3 extends AutoPathStrategy {
     public static final VisionAimManager.ShootingProfile SecondShotProfile = new VisionAimManager.ShootingProfile(0.00, 0.00, 0.00); 
    public static final VisionAimManager.ShootingProfile ThirdShotProfile = new VisionAimManager.ShootingProfile(0.00, 0.00,0.00); 
    public static final VisionAimManager.ShootingProfile ForthShotProfile = new VisionAimManager.ShootingProfile(0, 0, 0); 
   
    private SwerveRequest.PointWheelsAt pointWheels = new SwerveRequest.PointWheelsAt().withModuleDirection(Rotation2d.fromDegrees(0));


   public static void registerAutoPaths(PathRegistry registry) {
    registry.registerPaths( "Front", "Front2", "Front3");

   }
 public SL1toSL3() {
    addCommands(
        Commands.parallel(
            new PrintStartInfo("SL1-SL3"),
            new InitializeAutoState(Degrees.of(0)),
            new EnableShooterCommand()
        ),

        //Shoot Preload
        Subsystems.poseManager.getPoseCommand(PoseManager.Pose.AutoShortShot),
        new WaitCommand(0.25),
        Subsystems.shooter.shootCmd(),
        Commands.parallel(
            new EnableShooterCommand(false),
            Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Drive)
        ),

        this.runAutoPath("Front"),
        new EnableShooterCommand(false),

        //Pickup/Shoot SL1 note
        // new WaitShooterHasNote().withTimeout(1.0),???
         Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Pickup),
         Commands.parallel(
         Commands.runOnce(() -> Subsystems.pivot.applyShootingProfile(SL1toSL3.SecondShotProfile)),
         new EnableShooterCommand()
             ),
            new WaitCommand(0.5),
           Subsystems.shooter.shootCmd()
    );
 }

        //Front 2
//        this.runAutoPath("Front2"),
//      Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Pickup),
//
//      Subsystems.poseManager.getPoseCommand(PoseManager.Pose.AutoShortShot),
//        new WaitCommand(0.25),
//       Subsystems.shooter.shootCmd(),
//        Commands.parallel(
//            new EnableShooterCommand(false),
//            Subsystems.poseManager.getPoseCommand(PoseManager.Pose.Drive)
//        ),
//      this.runAutoPath("Front3"),
//         new WaitIntakeInPosition(IntakePivot.IntakePosition.Zero).withTimeout(1.0),
//                new WaitIntakeHasNoteCommand().withTimeout(1.0),
//                Subsystems.poseManager.getPoseCommand(PoseManager.Pose.FeedNoteToShooter),
//                new WaitShooterHasNote().withTimeout(1.0),
//                Commands.parallel(
//                        Commands.runOnce(() -> Subsystems.pivot.applyShootingProfile(SL1toSL3.ForthShotProfile)),
//                        new EnableShooterCommand()
//                ),
//                new WaitCommand(0.5),
//                Subsystems.shooter.shootCmd()
//        );
// }
            public Command waitForPositionThenShoot() {
                return new SequentialCommandGroup(
);
            }

        

 }


