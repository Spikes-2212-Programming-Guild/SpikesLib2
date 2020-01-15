package com.spikes2212.utils;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

public class SpikesPigeon {
    PigeonIMU pigeon;

    public SpikesPigeon(int canPort) {
        pigeon = new PigeonIMU(canPort);
    }

    public SpikesPigeon(TalonSRX talonSRX) {
        pigeon = new PigeonIMU(talonSRX);
    }

    public double getX() {
        double[] arr = new double[3];
        pigeon.getAccumGyro(arr);
        return arr[0];
    }

    public double getY() {
        double[] arr = new double[3];
        pigeon.getAccumGyro(arr);
        return arr[1];
    }

    public double getZ() {
        double[] arr = new double[3];
        pigeon.getAccumGyro(arr);
        return arr[2];
    }

    public double getYaw() {
        double[] arr = new double[3];
        pigeon.getYawPitchRoll(arr);
        return arr[0];
    }

    public void setYaw(double yaw) {
        pigeon.setYaw(yaw);
    }

    public void calibrate() {
        pigeon.enterCalibrationMode(PigeonIMU.CalibrationMode.BootTareGyroAccel);
        setYaw(0);
    }

    public void calibrate(double yaw) {
        pigeon.enterCalibrationMode(PigeonIMU.CalibrationMode.BootTareGyroAccel);
        setYaw(yaw);
    }

}
