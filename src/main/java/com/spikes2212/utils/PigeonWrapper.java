package com.spikes2212.utils;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

public class PigeonWrapper {
    public double[] arr = new double[3];
    PigeonIMU pigeon;

    public PigeonWrapper(int canPort) {
        pigeon = new PigeonIMU(canPort);
    }

    public PigeonWrapper(TalonSRX talonSRX) {
        pigeon = new PigeonIMU(talonSRX);
    }

    public double getX() {
        pigeon.getAccumGyro(arr);
        return arr[0];
    }

    public double getY() {
        pigeon.getAccumGyro(arr);
        return arr[1];
    }

    public double getZ() {
        pigeon.getAccumGyro(arr);
        return arr[2];
    }

    public double getYaw() {
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
