// package frc.robot.commands;

// import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;
// import frc.robot.Subsystems;
// import frc.robot.subsystems.OLDSwerve;


// public class OLDTeleopSwerve extends CommandBase {    
//     private OLDSwerve s_Swerve;    
//     private DoubleSupplier translationSup;
//     private DoubleSupplier strafeSup;
//     private boolean strafeDeadband = true;
//     private DoubleSupplier rotationSup;
//     private BooleanSupplier robotCentricSup;
//     private BooleanSupplier lockAngleEnabledSup;
//     private DoubleSupplier lockAngleSup;
    
//     // Extended elevator constraints
//     private static final double MAX_TRANSLATION_SPEED = 4.0;    // m/s
//     private static final double DECEL_RATE = 0.07;
//     public static final double ROTATION_CLAMP_RAD_PER_SEC = 8;

//     private Translation2d lastSpeed = new Translation2d(0,0);

//     private SlewRateLimiter transLimiter = new SlewRateLimiter(0.5);
//     private SlewRateLimiter strafeLimiter = new SlewRateLimiter(0.5);

//     public OLDTeleopSwerve(
//             OLDSwerve s_Swerve, 
//             DoubleSupplier translationSup,
//             DoubleSupplier strafeSup,
//             DoubleSupplier rotationSup, 
//             BooleanSupplier robotCentricSup, 
//             BooleanSupplier lockAngleEnabledSup, 
//             DoubleSupplier lockAngleSup) {
//         this.s_Swerve = s_Swerve;
//         addRequirements(s_Swerve);

//         this.translationSup = translationSup;
//         this.strafeSup = strafeSup;
//         this.rotationSup = rotationSup;
//         this.robotCentricSup = robotCentricSup;
//         this.lockAngleEnabledSup = lockAngleEnabledSup;
//         this.lockAngleSup = lockAngleSup;

//         SmartDashboard.putNumber("RotationClamp", ROTATION_CLAMP_RAD_PER_SEC);
//     }

//     public void setStrafeSupplier(DoubleSupplier supplier, boolean strafeDeadband) {
//         this.strafeSup = supplier;
//         this.strafeDeadband = strafeDeadband;
//     }

//     @Override
//     public void execute() {
//         /* Get Values, Deadband*/
//         double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
//         double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), strafeDeadband? Constants.stickDeadband: 0);
//         double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

//         // Define maximum speeds
//         double maxSpeed = Subsystems.elevator.isElevatorExtended() ? MAX_TRANSLATION_SPEED : Constants.Swerve.maxSpeed;
//         maxSpeed = Constants.Swerve.maxSpeed;  // Temporary Testing for relaxing fine alignment
//         final double maxRotation = Subsystems.elevator.isElevatorExtended() ? 4 : Constants.Swerve.maxAngularVelocity;
//         rotationVal *= maxRotation;

//         // Target swerve velocity state
//         if (Subsystems.elevator.isElevatorExtended()) {
//             translationVal = transLimiter.calculate(translationVal);
//             strafeVal = strafeLimiter.calculate(strafeVal);
//         } else {
//             transLimiter.reset(translationVal);
//             strafeLimiter.reset(strafeVal);
//         }
//         Translation2d targetSpeed = new Translation2d(translationVal, strafeVal).times(maxSpeed);
//         final Translation2d swerveSpeed = targetSpeed;
//         // if (Subsystems.elevator.isElevatorExtended() && 
//         //     (lastSpeed.getX() >= targetSpeed.getX() || lastSpeed.getY() >= targetSpeed.getY())) {
//         //         // Interpolate slowly towards target if we are extended and going faster than target velocity
//         //     swerveSpeed = new Translation2d(
//         //         MathUtil.interpolate(lastSpeed.getX(), targetSpeed.getX(), DECEL_RATE),
//         //         MathUtil.interpolate(lastSpeed.getY(), targetSpeed.getY(), DECEL_RATE)
//         //     );
//         // } else {
//         //     swerveSpeed = targetSpeed;
//         // }


//         // Determine if we need to lock to angle
//         if (lockAngleEnabledSup.getAsBoolean()) {
//             rotationVal = Math.toRadians(
//                 Subsystems.swerveSubsystem.getRotationController().calculate(
//                     Subsystems.swerveSubsystem.getYaw().getDegrees(),
//                     lockAngleSup.getAsDouble()));
//             double clamp = SmartDashboard.getNumber("RotationClamp", ROTATION_CLAMP_RAD_PER_SEC);
//             rotationVal = MathUtil.clamp(rotationVal, -clamp, clamp);
//         }

//         lastSpeed = swerveSpeed;

//         /* Drive */
//         s_Swerve.drive(
//             swerveSpeed, 
//             rotationVal, 
//             !robotCentricSup.getAsBoolean(), 
//             true
//         );
//     }
// }