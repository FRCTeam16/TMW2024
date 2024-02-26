package frc.robot.generated;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.util.Units;
import frc.robot.config.RobotConfiguration;
import frc.robot.CommandSwerveDrivetrain;

public class TunerConstants {
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.05)
        .withKS(0).withKV(1.5).withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(3).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;


    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double kSlipCurrentA = 100.0;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double kSpeedAt12VoltsMps = RobotConfiguration.config.swerve().speedAt12v();

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = RobotConfiguration.config.swerve().coupleRatio();

    private static final double kDriveGearRatio = RobotConfiguration.config.swerve().driveGearRatio();
    private static final double kSteerGearRatio = RobotConfiguration.config.swerve().steerGearRatio();
    private static final double kWheelRadiusInches = 1.9;

    private static final boolean kSteerMotorReversed = true;
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final String kCANbusName = RobotConfiguration.config.canbusName();
    private static final int kPigeonId = RobotConfiguration.config.pigeonId();


    // These are only used for simulation
    private static final double kSteerInertia = 0.00001;
    private static final double kDriveInertia = 0.001;

    private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withPigeon2Id(kPigeonId)
            .withCANbusName(kCANbusName);

    private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withSteerMotorInverted(kSteerMotorReversed);


    // Front Left
    private static final int kFrontLeftDriveMotorId = RobotConfiguration.config.swerve().fl().driveId();
    private static final int kFrontLeftSteerMotorId = RobotConfiguration.config.swerve().fl().steerId();
    private static final int kFrontLeftEncoderId = RobotConfiguration.config.swerve().fl().encoderId();
    private static final double kFrontLeftEncoderOffset = RobotConfiguration.config.swerve().fl().encoderOffset();

    private static final double kFrontLeftXPosInches = 11.375;
    private static final double kFrontLeftYPosInches = 11.375;

    // Front Right
    private static final int kFrontRightDriveMotorId = RobotConfiguration.config.swerve().fr().driveId();
    private static final int kFrontRightSteerMotorId = RobotConfiguration.config.swerve().fr().steerId();
    private static final int kFrontRightEncoderId = RobotConfiguration.config.swerve().fr().encoderId();
    private static final double kFrontRightEncoderOffset = RobotConfiguration.config.swerve().fr().encoderOffset();

    private static final double kFrontRightXPosInches = 11.375;
    private static final double kFrontRightYPosInches = -11.375;

    // Back Left
    private static final int kBackLeftDriveMotorId = RobotConfiguration.config.swerve().rl().driveId();
    private static final int kBackLeftSteerMotorId = RobotConfiguration.config.swerve().rl().steerId();
    private static final int kBackLeftEncoderId = RobotConfiguration.config.swerve().rl().encoderId();
    private static final double kBackLeftEncoderOffset = RobotConfiguration.config.swerve().rl().encoderOffset();

    private static final double kBackLeftXPosInches = -11.375;
    private static final double kBackLeftYPosInches = 11.375;

    // Back Right
    private static final int kBackRightDriveMotorId = RobotConfiguration.config.swerve().rr().driveId();
    private static final int kBackRightSteerMotorId = RobotConfiguration.config.swerve().rr().steerId();
    private static final int kBackRightEncoderId = RobotConfiguration.config.swerve().rr().encoderId();
    private static final double kBackRightEncoderOffset = RobotConfiguration.config.swerve().rr().encoderOffset();

    private static final double kBackRightXPosInches = -11.375;
    private static final double kBackRightYPosInches = -11.375;


    private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
    private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
    private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
    private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);

    public static final CommandSwerveDrivetrain DriveTrain = new CommandSwerveDrivetrain(DrivetrainConstants, FrontLeft,
            FrontRight, BackLeft, BackRight);
}
