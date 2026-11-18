package com.spikes2212.util.gyro;

import com.spikes2212.util.Simulatable;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.DoubleSupplier;

/**
 * A simulated gyroscope that integrates angular velocity over time.
 *
 * @author Adi Yekutiel
 * @see Gyro
 * @see Simulatable
 */
public class SimulatedPigeonWrapper implements Gyro {

    private static final double DEFAULT_VALUE = 0.0;

    private final DoubleSupplier yawVelocitySupplier;

    private double yaw;
    private double pitch;
    private double roll;

    public SimulatedPigeonWrapper(DoubleSupplier yawVelocitySupplier) {
        this.yawVelocitySupplier = yawVelocitySupplier;
        this.yaw = DEFAULT_VALUE;
        this.pitch = DEFAULT_VALUE;
        this.roll = DEFAULT_VALUE;
    }

    public SimulatedPigeonWrapper(int canId, DoubleSupplier yawVelocitySupplier) {
        this(yawVelocitySupplier);
    }

    public SimulatedPigeonWrapper(int canId) {
        this(() -> DEFAULT_VALUE);
    }

    public SimulatedPigeonWrapper() {
        this(() -> DEFAULT_VALUE);
    }

    @Override
    public void reset() {
        setYaw(DEFAULT_VALUE);
    }

    @Override
    public void calibrate(double newAngle) {
        setYaw(newAngle);
    }

    @Override
    public void calibrate() {
        calibrate(DEFAULT_VALUE);
    }

    @Override
    public double getYaw() {
        return yaw;
    }

    public double getPitch() {
        return pitch;
    }

    public double getRoll() {
        return roll;
    }

    @Override
    public double getX() {
        return getRoll();
    }

    @Override
    public double getY() {
        return getPitch();
    }

    @Override
    public double getZ() {
        return getYaw();
    }

    public void setYaw(double yawDegrees) {
        this.yaw = yawDegrees;
    }

    public void addYaw(double deltaDegrees) {
        this.yaw += deltaDegrees;
    }

    public void setPitch(double pitchDegrees) {
        this.pitch = pitchDegrees;
    }

    public void setRoll(double rollDegrees) {
        this.roll = rollDegrees;
    }

    @Override
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getYaw());
    }

    @Override
    public void update(double dtSeconds) {
        addYaw(yawVelocitySupplier.getAsDouble() * dtSeconds);
    }
}
