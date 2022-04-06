package com.spikes2212.util;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.interfaces.Gyro;

/**
 * A wrapper class for the pigeon IMU sensor.
 *
 * @author Tal Sitton
 */
public class PigeonWrapper implements Gyro {

    public enum RotationAxis {
        X, Y, Z;
    }

    protected final double[] values = new double[3];
    protected final PigeonIMU pigeon;

    private final RotationAxis axis;

    public PigeonWrapper(int canPort, RotationAxis rotationAxis) {
        this.pigeon = new PigeonIMU(canPort);
        this.axis = rotationAxis;
    }

    public PigeonWrapper(TalonSRX talonSRX, RotationAxis rotationAxis) {
        this.pigeon = new PigeonIMU(talonSRX);
        this.axis = rotationAxis;
    }

    public PigeonWrapper(int canPort) {
        this(canPort, RotationAxis.Z);
    }

    public PigeonWrapper(TalonSRX talonSRX) {
        this(talonSRX, RotationAxis.Z);
    }

    /**
     * Calibrates the pigeon based on the yaw sent.
     *
     * @param yaw the yaw the pigeon shall be calibrated to
     */
    public void calibrate(double yaw) {
        pigeon.enterCalibrationMode(PigeonIMU.CalibrationMode.BootTareGyroAccel);
        setYaw(yaw);
    }

    /**
     * Calibrates the pigeon wrapper to yaw 0.
     */
    public void calibrate() {
        calibrate(0);
    }

    @Override
    public void reset() {
        calibrate();
    }

    /**
     * Don't.
     */
    @Override
    public void close() throws UnsupportedOperationException {
        throw new UnsupportedOperationException("Why did you feel the need to close the pigeon");
    }

    /**
     * @return the heading of the robot in degrees
     */
    @Override
    public double getAngle() {
        if (axis == RotationAxis.X)
            return getPitch();
        if (axis == RotationAxis.Y)
            return getRoll();
        return getYaw();
    }

    /**
     * @return the angle change rate in degrees per second
     */
    @Override
    public double getRate() {
        pigeon.getRawGyro(values);
        if (axis == RotationAxis.X)
            return values[0];
        if (axis == RotationAxis.Y)
            return values[1];
        return values[2];
    }

    /**
     * @return the X axis from the gyro
     */
    public double getX() {
        pigeon.getAccumGyro(values);
        return values[0];
    }

    /**
     * @return the Y axis from the gyro
     */
    public double getY() {
        pigeon.getAccumGyro(values);
        return values[1];
    }

    /**
     * @return the Z axis from the gyro
     */
    public double getZ() {
        pigeon.getAccumGyro(values);
        return values[2];
    }

    /**
     * <p>If you don't know what yaw is, see <a href=https://letmegooglethat.com/?q=what+is+yaw>here</a>.</p>
     *
     * @return the yaw
     */
    public double getYaw() {
        pigeon.getYawPitchRoll(values);
        return values[0];
    }

    /**
     * @param yaw if you don't know what yaw is, see <a href=https://letmegooglethat.com/?q=what+is+yaw>here</a>
     */
    public void setYaw(double yaw) {
        pigeon.setYaw(yaw);
    }

    public double getPitch() {
        pigeon.getYawPitchRoll(values);
        return values[1];
    }

    public double getRoll() {
        pigeon.getYawPitchRoll(values);
        return values[2];
    }
}
