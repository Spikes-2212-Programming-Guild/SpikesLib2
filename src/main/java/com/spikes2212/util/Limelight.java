package com.spikes2212.util;

import edu.wpi.first.networktables.*;

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
     * Whether the limelight has any valid targets.
     */
    private NetworkTableEntry tv;

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

    /**
     * The skew or rotation.
     */
    private NetworkTableEntry ts;

    /**
     * The pipeline's latency contribution.
     */
    private NetworkTableEntry tl;

    /**
     * Side length of the shortest side of the fitted bounding box.
     */
    private NetworkTableEntry tshort;

    /**
     * Side length of the longest side of the fitted bounding box.
     */
    private NetworkTableEntry tlong;

    /**
     * Horizontal side length of the rough bounding box.
     */
    private NetworkTableEntry thor;

    /**
     * Vertical side length of the rough bounding box.
     */
    private NetworkTableEntry tvert;

    public Limelight(String limelightName) {

        networkTable = NetworkTableInstance.getDefault().getTable(limelightName);

        ledMode = networkTable.getEntry("ledMode");
        camMode = networkTable.getEntry("camMode");
        pipeline = networkTable.getEntry("pipeline");

        tv = networkTable.getEntry("tv");
        tx = networkTable.getEntry("tx");
        ty = networkTable.getEntry("ty");
        ta = networkTable.getEntry("ta");
        ts = networkTable.getEntry("ts");
        tl = networkTable.getEntry("tl");
        tshort = networkTable.getEntry("tshort");
        tlong = networkTable.getEntry("tlong");
        thor = networkTable.getEntry("thor");
        tvert = networkTable.getEntry("tvert");
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

    public boolean getHasValidTargets() {
        return tv.getDouble(0) == 1;
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

    public double getSkew() {
        return ts.getDouble(0);
    }

    public double getPipelineLatency() {
        return tl.getDouble(0);
    }

    public double getShortestFittedBoundingBoxSide() {
        return tshort.getDouble(0);
    }

    public double getLongestFittedBoundingBoxSide() {
        return tlong.getDouble(0);
    }

    public double getRoughBoundingBoxHorizontalSide() {
        return thor.getDouble(0);
    }

    public double getRoughBoundingBoxVerticalSide() {
        return tvert.getDouble(0);
    }

}
