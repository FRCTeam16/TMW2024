package frc.robot.commands.auto;

// import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.Subsystems;

// FIXME: Needs to be reimplemented
public class ProfiledDistanceDriveCommand extends Command {

//   public static boolean debug = true;  // controls writing sysouts

//   private double distanceThreshold = 0.05;
//   private double angle;
//   private double maxSpeed;
//   private double xdist;
//   private double ydist;

//   private Translation2d startPose;
//   private Translation2d targetPose;

//   private Constraints constraints;
//   private State goal;
//   private State currentState;
//   private double endSpeed = 0.0;
//   private boolean fieldCentric = true;

//   public BooleanSupplier stopFunction;  // optional stop function evaluated for early termination
//   public DoubleSupplier yTranslationFunction; // optional x-direction supplier, e.g. vision


//   /**
//    * Uses trapezoid profile to drive a distance
//    * @param angleDegrees
//    * @param speed - percent speed
//    * @param xMeters
//    * @param yMeters
//    */
//   public ProfiledDistanceDriveCommand(double angleDegrees, double speed, double xMeters, double yMeters) {
//     this.angle = angleDegrees;
//     this.maxSpeed = MathUtil.clamp(speed, -1, 1) * Constants.AutoConstants.kMaxSpeedMetersPerSecond;
//     this.xdist = xMeters;
//     this.ydist = yMeters;

//     this.constraints = new TrapezoidProfile.Constraints(
//       maxSpeed,
//       Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared
//     );

//     addRequirements(Subsystems.swerveSubsystem);
//   }

//   /**
//    * Overrides the default threshold for determining we are close enough
//    * @param threshold
//    * @return
//    */
//   public ProfiledDistanceDriveCommand withThreshold(double threshold) {
//     this.distanceThreshold = threshold;
//     return this;
//   }

//   public ProfiledDistanceDriveCommand withEndSpeed(double percentSpeed) {
//     this.endSpeed = MathUtil.clamp(percentSpeed, -1, 1) * Constants.AutoConstants.kMaxSpeedMetersPerSecond;
//     return this;
//   }

//   public ProfiledDistanceDriveCommand withRobotCentric() {
//     this.fieldCentric = false;
//     return this;
//   }

//   public ProfiledDistanceDriveCommand withConstraints(TrapezoidProfile.Constraints constraints) {
//     this.constraints = constraints;
//     return this;
//   }

//   public ProfiledDistanceDriveCommand withStopCondition(BooleanSupplier stopFunction) {
//     this.stopFunction = stopFunction;
//     return this;
//   }

//   public ProfiledDistanceDriveCommand withYSupplier(DoubleSupplier ySupplier) {
//     this.yTranslationFunction = ySupplier;
//     return this;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     this.startPose = Subsystems.swerveSubsystem.getPose().getTranslation();
//     var translation = new Translation2d(xdist, ydist);
//     if (!fieldCentric) {
//       translation = translation.rotateBy(Subsystems.swerveSubsystem.gyro.getGyroscopeRotation());
//     }
//     var xy_trans = startPose.plus(translation);
//     this.targetPose = new Translation2d(xy_trans.getX(), xy_trans.getY());

//     this.currentState = new TrapezoidProfile.State(0, 0); // assume zero initial velocity
//     this.goal = new TrapezoidProfile.State(startPose.getDistance(xy_trans), endSpeed);

//     if (!this.fieldCentric) {
//       this.angle = Subsystems.swerveSubsystem.getYaw().getDegrees();
//     }

//     System.out.println("****************> SDDC Initialize");
//     System.out.println("Current Pose: " + startPose);
//     System.out.println("Target Pose : " + targetPose);
//     System.out.println("<**************** SDDC Initialize");
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     var currentPose = Subsystems.swerveSubsystem.getPose().getTranslation();

//     // Use velocity from last state, but update out position
//     this.currentState = new TrapezoidProfile.State(currentPose.getDistance(startPose), this.currentState.velocity);
//     var profile = new TrapezoidProfile(constraints, goal, currentState);

//     // Calculate what our next position and velocity should be
//     this.currentState = profile.calculate(0.2);
    
//     var speed = currentState.velocity;

//     //
//     var driveAngle = Math.atan2(
//       targetPose.getY() - currentPose.getY(),
//       targetPose.getX() - currentPose.getX());

//     double vxMetersPerSecond = Math.abs(speed * Math.cos(driveAngle));
//     double vyMetersPerSecond = Math.abs(speed * Math.sin(driveAngle));
    
//     // TODO: may just need to add 90 to driveAngle and use true quadrant sign values
//     if (currentPose.getY() > targetPose.getY()) {
//       if (debug) System.out.println("*** inverting VYM: " + currentPose.getY() + " > " + targetPose.getY());
//       vyMetersPerSecond = -vyMetersPerSecond;
//     }
//     if (currentPose.getX() > targetPose.getX()) {
//       vxMetersPerSecond = -vxMetersPerSecond;
//     }

//     // Handle override
//     if (yTranslationFunction != null) {
//       double percentX = yTranslationFunction.getAsDouble();
//       if (debug) System.out.print("[PDDC] Original vxMetersPerSec: " + vyMetersPerSecond );
//       vyMetersPerSecond = percentX * Constants.Swerve.kMaxSpeedMetersPerSecond;
//       if (debug) System.out.println(" | Override VX = " + vyMetersPerSecond);
//     }

// /* 
// Test new
//     var driveAngle = Math.atan2(
//       targetPose.getX() - currentPose.getX()
//       targetPose.getY() - currentPose.getY());

//     double vxMetersPerSecond = speed * Math.cos(driveAngle);
//     double vyMetersPerSecond = speed * Math.sin(driveAngle);
// */

//     double clamp = SmartDashboard.getNumber("AutoRotationClamp", 6);

//     var twist = Subsystems.swerveSubsystem.getRotationController().calculate(
//         Subsystems.swerveSubsystem.gyro.getGyroscopeRotation().getDegrees(), angle);

//     double desiredRotation = MathUtil.clamp(Math.toRadians(twist), -clamp, clamp);
//     var desiredTranslation = new Translation2d(vxMetersPerSecond, vyMetersPerSecond);

    

    
//     if (debug) {
//       System.out.println(
//         "DA: " + driveAngle +
//         " | Yaw: " + Subsystems.swerveSubsystem.getYaw().getDegrees() + 
//         " | VX: " + vxMetersPerSecond / Constants.AutoConstants.kMaxSpeedMetersPerSecond + 
//         " | VY: " + vyMetersPerSecond / Constants.AutoConstants.kMaxSpeedMetersPerSecond +
//         " | TW: " + twist);
//         System.out.println("DT: " + desiredTranslation);
//         System.out.println("SPEED: " + speed + " | VXM: " + vxMetersPerSecond + " | VYM: " + vyMetersPerSecond);
//       }

//     Subsystems.swerveSubsystem.drive(
//       desiredTranslation, 
//       desiredRotation, 
//       fieldCentric, 
//       true);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {

//     if (this.stopFunction != null && this.stopFunction.getAsBoolean()) {
//       System.out.println("Stop Function Stopped exectuion");
//       return true;
//     }

//     var distance = Subsystems.swerveSubsystem.getPose().getTranslation().getDistance(targetPose);
//     if (debug) {
//       System.out.println("DIST: " + distance + " | T:" + targetPose + " | C:" + Subsystems.swerveSubsystem.getPose().getTranslation());
//     }
//     if (distance < distanceThreshold) {
//       System.out.println(("=======> SDDC HIT DISTANCE <========"));
//     }
//     System.out.println("->End PDDC Scan<-");
//     return (distance < distanceThreshold);
//   }
}
