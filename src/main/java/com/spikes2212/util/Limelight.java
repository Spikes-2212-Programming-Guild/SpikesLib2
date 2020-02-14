package com.spikes2212.util;

import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {

    public enum LedMode {

        BY_PIPELINE(0),
        OFF(1),
        BLINK(2),
        ON(3);

        int state;

        LedMode(int state) {
            this.state = state;
        }

        int getState() {
            return state;
        }
    }

    public enum CameraMode {
        VISION_PROCESSING(0),
        DRIVERS(1);

        int mode;

        CameraMode(int mode) {
            this.mode = mode;
        }

        int getMode() {
            return mode;
        }
    }

    public Limelight() {

    }

    public void changeLedMode(LedMode mode) {
        NetworkTableInstance.getDefault().getTable("limelight").
                getEntry("ledMode").setNumber(mode.getState());
    }

    public void changeCameraMode(CameraMode mode) {
        NetworkTableInstance.getDefault().getTable("limelight").
                getEntry("camMode").setNumber(mode.getMode());
    }

    public void setPipeline(int pipelineNum) {
        NetworkTableInstance.getDefault().getTable("limelight").
                getEntry("pipeline").setNumber(pipelineNum);
    }

    public double getHorizontalAngleToTarget() {
        return NetworkTableInstance.getDefault().getTable("limelight").
                getEntry("tx").getDouble(0);
    }

    public double getVerticalAngleToTarget() {
        return NetworkTableInstance.getDefault().getTable("limelight").
                getEntry("ty").getDouble(0);

    }

    public double getTargetArea() {
        return NetworkTableInstance.getDefault().getTable("limelight").
                getEntry("ta").getDouble(0);

    }

}
