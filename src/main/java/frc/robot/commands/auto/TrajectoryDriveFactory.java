package frc.robot.commands.auto;
/* 
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Subsystems;
*/

public class TrajectoryDriveFactory {
/* 
    public static final TrajectoryConfig DEFAULT_TRAJECTORY_CONFIG = 
        new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

    public static SwerveControllerCommand createCommand(Trajectory trajectory) {
        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 
                Constants.AutoConstants.kIThetaController,
                Constants.AutoConstants.kDThetaController, 
                Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        
        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                trajectory,
                Subsystems.swerveSubsystem::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                Subsystems.swerveSubsystem::setModuleStates,
                Subsystems.swerveSubsystem);

        return swerveControllerCommand;
    }
    */
}
