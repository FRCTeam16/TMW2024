package frc.robot;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class MusicController implements Sendable {
    private final Orchestra orchestra = new Orchestra();

    public MusicController() {
        for (int i =0; i<4; i++) {
            TalonFX motor = Subsystems.swerveSubsystem.getModule(i).getDriveMotor();
            TalonFX steer = Subsystems.swerveSubsystem.getModule(i).getSteerMotor();
            AudioConfigs audio = new AudioConfigs().withAllowMusicDurDisable(true);
            motor.getConfigurator().apply(audio);
            orchestra.addInstrument(motor);
            orchestra.addInstrument(steer);
        }
        orchestra.loadMusic("/home/lvuser/deploy/music/lowrida.chrp");
    }

    public void play() {
        orchestra.play();
    }

    public void pause() {
        orchestra.pause();
    }

    public Command getPlayCommand() {
        return Commands.runOnce(this::play);
    }

    public Command getPauseommand() {
        return Commands.runOnce(this::pause);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("MusicController");
        builder.addBooleanProperty("Playing", this.orchestra::isPlaying, null);

    }
}
