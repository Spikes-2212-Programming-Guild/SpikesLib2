package com.spikes2212.util;

import edu.wpi.first.networktables.NetworkTable;
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

    private NetworkTable networkTable;

    public Limelight() {
        networkTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public void changeLedMode(LedMode mode) {
        networkTable.getEntry("ledMode").setNumber(mode.getState());
    }

    public void changeCameraMode(CameraMode mode) {
        networkTable.getEntry("camMode").setNumber(mode.getMode());
    }

    public void setPipeline(int pipelineNum) {
        networkTable.getEntry("pipeline").setNumber(pipelineNum);
    }

    public double getHorizontalAngleToTarget() {
        return networkTable.getEntry("tx").getDouble(0);
    }

    public double getVerticalAngleToTarget() {
        return networkTable.getEntry("ty").getDouble(0);

    }

    public double getTargetArea() {
        return networkTable.getEntry("ta").getDouble(0);

    }

}
