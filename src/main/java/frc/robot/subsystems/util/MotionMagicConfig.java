package frc.robot.subsystems.util;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

public class MotionMagicConfig {
    public double kS = 0.0;
    public double kG = 0.0;
    private double velocity = 30;
    private double acceleration = 250;
    private double jerk = 0;

    public double getkS() {
        return kS;
    }

    public void setkS(double kS) {
        this.kS = kS;
    }

    public double getkG() {
        return this.kG;
    }

    public void setkG(double kG) {
        this.kG = kG;
    }

    public double getAcceleration() {
        return acceleration;
    }

    public void setAcceleration(double acceleration) {
        this.acceleration = acceleration;
    }

    public double getVelocity() {
        return velocity;
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    public double getJerk() {
        return jerk;
    }

    public void setJerk(double jerk) {
        this.jerk = jerk;
    }

    public void updateMotionMagicConfig(MotionMagicConfigs config) {
        config.withMotionMagicAcceleration(this.acceleration)
                .withMotionMagicCruiseVelocity(this.velocity)
                .withMotionMagicJerk(this.jerk);
    }

    public void updateSlot0Config(Slot0Configs config) {
        config.withKG(this.kG).withKS(this.kS);
    }
}
