/*package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.subsystems.DMS.DriveInfo;
import frc.robot.subsystems.gyro.BSGyro;
import frc.robot.subsystems.gyro.PigeonGyro;

public class OLDSwerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public final BSGyro gyro;
    public final RotationController rotationController;

    public OLDSwerve() {
         gyro = new PigeonGyro(Constants.Swerve.pigeonID);
         rotationController = new RotationController();
        
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
    }

    public void tryForceZeroWheels() {
        System.out.println("=========> TRY FORCE ZERO WHEELS !!!!");
        for (int m=0;m<mSwerveMods.length;m++) {
            for (int i=0; i < 3; i++) {
                SwerveModule mod = mSwerveMods[m];
                ErrorCode error = mod.forceResetToAbsoluteAngle();
                System.out.println("*** TRY FORCE ZERO MOD[" + m + "]  Try(" + i + ") = " + error);
                if (error == null || error.value == 0) {
                    System.out.println("SUCCESS");
                    continue;
                } else if (error != null) {
                    System.out.println("ERROR: " + error.toString() + " | " + error.value);
                }
                Timer.delay(0.001);
            }
        }
        System.out.println("<========= TRY FORCE ZERO WHEELS !!!!");
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    // Used by SwerveControllerCommand in Auto 
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            // double start = System.nanoTime();
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
            // System.out.println("Module[" + mod.moduleNumber + "] setModuleState: " + (System.nanoTime() - start) + " ns");
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public void hardResetOdometry(double yaw) {
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, Rotation2d.fromDegrees(yaw), getModulePositions());
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.zeroGyroscope();
    }

    public Rotation2d getYaw() {
        // return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
        return (Constants.Swerve.invertGyro) ? 
            Rotation2d.fromDegrees(360 - gyro.getGyroscopeRotation().getDegrees()) : 
            gyro.getGyroscopeRotation();

    }

    public RotationController getRotationController() {
        return this.rotationController;
    } 

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

        SmartDashboard.putNumber("NavX", gyro.getGyroscopeRotation().getDegrees());

        SmartDashboard.putData("SwerveRotationController", this.rotationController);

        SmartDashboard.putNumber("PoseX", this.getPose().getTranslation().getX());
        SmartDashboard.putNumber("PoseY", this.getPose().getTranslation().getY());
    }

    public void DMSDrive(double speed) {
        for(SwerveModule mod : mSwerveMods) {
            mod.setDriveMotorManually(speed);
        }
    }

    public void DMSSteer(double speed) {
        for(SwerveModule mod : mSwerveMods) {
            mod.setWheelAnglePercentManually(speed);
        }
    }

    public DriveInfo<Double> getDriveOutputCurrent() {
        return new DriveInfo<Double>(
            mSwerveMods[0].getDriveMotorCurrent(),
            mSwerveMods[1].getDriveMotorCurrent(),
            mSwerveMods[2].getDriveMotorCurrent(),
            mSwerveMods[3].getDriveMotorCurrent());
    }

    public DriveInfo<Double> getDriveVelocity() {
        return new DriveInfo<Double> (
            mSwerveMods[0].getState().speedMetersPerSecond,
            mSwerveMods[1].getState().speedMetersPerSecond,
            mSwerveMods[2].getState().speedMetersPerSecond,
            mSwerveMods[3].getState().speedMetersPerSecond);
    }

    public DriveInfo<Double> getSteerOutputCurrent() {
        return new DriveInfo<Double>(
            mSwerveMods[0].getAngleMotorCurrent(),
            mSwerveMods[1].getAngleMotorCurrent(),
            mSwerveMods[2].getAngleMotorCurrent(),
            mSwerveMods[3].getAngleMotorCurrent());
    }

    public DriveInfo<Double> getSteerVelocity() {
        return new DriveInfo<Double>(
            mSwerveMods[0].getAngleMotorVelocity(),
            mSwerveMods[1].getAngleMotorVelocity(),
            mSwerveMods[2].getAngleMotorVelocity(),
            mSwerveMods[3].getAngleMotorVelocity());
    }
}
*/