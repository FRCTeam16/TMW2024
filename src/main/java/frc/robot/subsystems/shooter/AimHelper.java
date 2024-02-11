package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

@Deprecated  public class AimHelper implements Sendable {
    private boolean enabled = false;
    private final String name;
    private final TalonFX motor;

    public enum AimPosition {
        Zero(0),
        sampleText(100);  // sample text

        final int setPoint;
        private AimPosition(int i){
            this.setPoint = i;
        }
    }

    private AimPosition currenPosition = AimPosition.Zero;
    private boolean positionChanged = false;

    private final MotionMagicDutyCycle aimDrive_Request = new MotionMagicDutyCycle(0.0);


    public AimHelper(String parent, String name, TalonFX motor) {
        this.name = name;
        this.motor = motor;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public boolean isEnabled() {
        return enabled;
    }

    private String getCurrentPosition() {
        return this.currenPosition.name();
    }

    public void periodic() {
        if(positionChanged){
            // update position
            // TODO add constraints using .withLimitForward etc.
            motor.setControl(aimDrive_Request.withPosition(currenPosition.setPoint));
            positionChanged = false;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty(this.name + "/Enabled", this::isEnabled, this::setEnabled);
        builder.addStringProperty(this.name + "/CurrentPosition", this::getCurrentPosition, null);
    }
}
