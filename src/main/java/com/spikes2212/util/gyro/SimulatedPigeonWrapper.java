package com.spikes2212.util.gyro;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import java.util.function.DoubleSupplier;

/**
 * A simulated wrapper for a Pigeon gyroscope using {@link Pigeon2SimState}.
 *
 * @author Adi Yekutiel
 * @see Gyro
 */
public class SimulatedPigeonWrapper implements Gyro {

    private static final double INITIAL_VALUE = 0.0;

    private final Pigeon2 pigeon;
    private final Pigeon2SimState simState;
    private final DoubleSupplier yawVelocityDegreesPerSecSupplier;

    public SimulatedPigeonWrapper(int deviceId, DoubleSupplier yawVelocityDegreesPerSecSupplier) {
        this.pigeon = new Pigeon2(deviceId);
        this.simState = pigeon.getSimState();
        this.yawVelocityDegreesPerSecSupplier = yawVelocityDegreesPerSecSupplier;
    }

    public SimulatedPigeonWrapper(int deviceId) {
        this(deviceId, () -> INITIAL_VALUE);
    }

    @Override
    public void reset() {
        setYaw(INITIAL_VALUE);
    }

    @Override
    public void calibrate(double yaw) {
        setYaw(yaw);
    }

    @Override
    public void calibrate() {
        calibrate(INITIAL_VALUE);
    }

    @Override
    public void update(double dtSeconds) {
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        simState.addYaw(yawVelocityDegreesPerSecSupplier.getAsDouble() * dtSeconds);
    }

    @Override
    public double getX() {
        return pigeon.getRoll().getValueAsDouble();
    }

    public void setX(double rollDegrees) {
        simState.setRoll(rollDegrees);
    }

    @Override
    public double getY() {
        return pigeon.getPitch().getValueAsDouble();
    }

    public void setY(double pitchDegrees) {
        simState.setPitch(pitchDegrees);
    }

    @Override
    public double getZ() {
        return getYaw();
    }

    @Override
    public double getYaw() {
        return pigeon.getYaw().getValueAsDouble();
    }

    @Override
    public void setYaw(double yaw) {
        pigeon.setYaw(yaw);
        simState.setRawYaw(yaw);
    }

    @Override
    public Rotation2d getRotation2d() {
        return pigeon.getRotation2d();
    }
}
