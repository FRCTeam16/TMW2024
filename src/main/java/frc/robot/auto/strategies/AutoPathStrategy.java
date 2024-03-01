package frc.robot.auto.strategies;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems;
import frc.robot.subsystems.util.BSLogger;

import java.util.function.Supplier;

public class AutoPathStrategy extends SequentialCommandGroup {

    private final SwerveRequest.PointWheelsAt pointWheels = new SwerveRequest.PointWheelsAt().withModuleDirection(Rotation2d.fromDegrees(0));

    static Command writeLog(String owner, String message) {
        return Commands.runOnce(() -> BSLogger.log(owner, message));
    }

    static Command writeLog(String owner, Supplier<String> message) {
        return Commands.runOnce(() -> BSLogger.log(owner, message.get()));
    }

    Command pointWheelsAtCmd(Measure<Angle> angle) {
        return Commands.runOnce(() -> Subsystems.swerveSubsystem.setControl(pointWheels.withModuleDirection(Rotation2d.fromDegrees(angle.in(Units.Degrees)))));
    }

    public Command runAutoPath(String pathName) {
        return Subsystems.autoManager.getAutoPath(pathName);
    }
}
