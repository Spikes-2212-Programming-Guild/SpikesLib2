package com.spikes2212.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * A wrapper and interface for the limelight device.
 *
 * @author yuval levy
 */
public class Limelight {

    public enum LedMode {

        BY_PIPELINE(0),
        OFF(1),
        BLINK(2),
        ON(3);

        int mode;

        LedMode(int mode) {
            this.mode = mode;
        }

        int getMode() {
            return mode;
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

    /**
     * The limelight's network table.
     */
    private NetworkTable networkTable;

    /**
     * The limelight's led mode
     * Represented by the enum {@link LedMode}
     */
    private NetworkTableEntry ledMode;

    /**
     * The limelight's cam mode
     * Represented by the enum {@link CameraMode}
     */
    private NetworkTableEntry camMode;

    /**
     * The limelight's current pipeline index.
     * Index's value should be between 0 and 9
     */
    private NetworkTableEntry pipeline;

    /**
     * The horizontal angle to the target the limelight sees.
     */
    private NetworkTableEntry tx;

    /**
     * The vertical angle to the target the limelight sees.
     */
    private NetworkTableEntry ty;

    /**
     * The area of the target the limelight sees.
     */
    private NetworkTableEntry ta;

    public Limelight(String limelightName) {

        networkTable = NetworkTableInstance.getDefault().getTable(limelightName);

        ledMode = networkTable.getEntry("ledMode");
        camMode = networkTable.getEntry("camMode");
        pipeline = networkTable.getEntry("pipeline");

        tx = networkTable.getEntry("tx");
        ty = networkTable.getEntry("ty");
        ta = networkTable.getEntry("ta");
    }

    public Limelight() {
        this("limelight");
    }

    public void setLedMode(LedMode mode) {
        ledMode.setNumber(mode.getMode());
    }

    public void setCameraMode(CameraMode mode) {
        camMode.setNumber(mode.getMode());
    }

    public void setPipeline(int pipelineNum) {
        pipeline.setNumber(pipelineNum);
    }

    public double getHorizontalAngleToTarget() {
        return tx.getDouble(0);
    }

    public double getVerticalAngleToTarget() {
        return ty.getDouble(0);
    }

    public double getTargetArea() {
        return ta.getDouble(0);
    }

}
