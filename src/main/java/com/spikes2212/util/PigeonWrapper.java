package com.spikes2212.util;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

/**
 * A wrapper for phoenix's {@link PigeonIMU}.
 */
public class PigeonWrapper {
    /**
     * The x, y and z values of the IMU.
     */
    public double[] values = new double[3];

    /**
     * The {@link PigeonIMU} this wrapper wraps.
     */
    private PigeonIMU pigeon;

    public PigeonWrapper(int canPort) {
        pigeon = new PigeonIMU(canPort);
    }

    public PigeonWrapper(TalonSRX talonSRX) {
        pigeon = new PigeonIMU(talonSRX);
    }

    public double getX() {
        pigeon.getAccumGyro(values);
        return values[0];
    }

    public double getY() {
        pigeon.getAccumGyro(values);
        return values[1];
    }

    public double getZ() {
        pigeon.getAccumGyro(values);
        return values[2];
    }

    public double getYaw() {
        pigeon.getYawPitchRoll(values);
        return values[0];
    }

    public void setYaw(double yaw) {
        pigeon.setYaw(yaw);
    }

    /**
     * Calibrate the {@link PigeonIMU}.
     */
    public void calibrate() {
        pigeon.enterCalibrationMode(PigeonIMU.CalibrationMode.BootTareGyroAccel);
        setYaw(0);
    }

    /**
     * Calibrate the {@link PigeonIMU}.
     *
     * @param yaw the current yaw of the {@link PigeonIMU}.
     */
    public void calibrate(double yaw) {
        pigeon.enterCalibrationMode(PigeonIMU.CalibrationMode.BootTareGyroAccel);
        setYaw(yaw);
    }
}
