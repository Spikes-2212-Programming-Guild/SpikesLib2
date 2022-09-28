package com.spikes2212.util;

import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * This class is a limelight wrapper.<br>
 * <b>This class assumes you are using the 960x720 processing resolution on the limelight.</b>
 *
 * @author Yotam Yizhar
 */
public class Limelight {

    protected static RootNamespace rootNamespace = new RootNamespace("limelight values");
    protected static NetworkTableInstance table;

    public Limelight() {
        rootNamespace.putBoolean("is on target", this::isOnTarget);
        rootNamespace.putNumber("horizontal offset from target in degrees", this::getHorizontalOffsetFromTargetInDegrees);
        rootNamespace.putNumber("vertical offset from target in degrees", this::getVerticalOffsetFromTargetInDegrees);
        rootNamespace.putNumber("target screen fill percent", this::getTargetAreaPercentage);
        rootNamespace.putNumber("pipeline latency", this::getTargetLatency);
    }

    public void periodic() {
        rootNamespace.update();
    }

    /**
     * @return whether a target is detected by the limelight
     */
    public boolean isOnTarget() {
        return getValue("tv").getDouble(0) == 1;
    }

    /**
     * @return the current limelight's pipeline
     */
    public int getPipeline() {
        return (int) getValue("getpipe").getNumber(0);
    }

    /**
     * @return the horizontal offset from crosshair to target (-27 degrees to 27 degrees)
     */
    public double getHorizontalOffsetFromTargetInDegrees() {
        return getValue("tx").getDouble(0.00);
    }

    /**
     * @deprecated Use {@link #getHorizontalOffsetFromTargetInDegrees()}.
     */
    @Deprecated(since = "2022", forRemoval = true)
    public double getHorizontalOffsetFromTarget() {
        return getHorizontalOffsetFromTargetInDegrees();
    }

    /**
     * @return the raw horizontal offset from crosshair to target in pixels (-1 to 1)
     */
    public double getHorizontalOffsetFromTargetInPixels() {
        return getValue("tx0").getDouble(0.00);
    }

    /**
     * @return the vertical offset from crosshair to target (-20.5 degrees to 20.5 degrees)
     */
    public double getVerticalOffsetFromTargetInDegrees() {
        return getValue("ty").getDouble(0.00);
    }

    /**
     * @deprecated Use {@link #getVerticalOffsetFromTargetInDegrees()}.
     */
    @Deprecated(since = "2022", forRemoval = true)
    public double getVerticalOffsetFromTarget() {
        return getVerticalOffsetFromTargetInDegrees();
    }

    /**
     * @return the raw vertical offset from crosshair to target in pixels (-1 to 1)
     */
    public double getVerticalOffsetFromTargetInPixels() {
        return getValue("ty0").getDouble(0.00);
    }

    /**
     * @return the area that the detected target takes up in total camera FOV (0% to 100%)
     */
    public double getTargetAreaPercentage() {
        return getValue("ta").getDouble(0.00);
    }

    /**
     * @return the target skew or rotation (-90 degrees to 0 degrees)
     */
    public double getTargetSkew() {
        return getValue("ts").getDouble(0.00);
    }

    /**
     * @return target latency (ms)
     */
    public double getTargetLatency() {
        return getValue("tl").getDouble(0.00);
    }

    /**
     * @return the target width in pixels (0 pixels to 720 pixels)
     */
    public double getTargetWidthInPixels() {
        return getValue("thor").getDouble(0.00);
    }

    /**
     * @return the target height in pixels (0 pixels to 960 pixels)
     */
    public double getTargetHeightInPixels() {
        return getValue("tvert").getDouble(0.00);
    }

    /**
     * Sets pipeline number (0-9 value).
     *
     * @param number pipeline number (0-9)
     */
    public void setPipeline(int number) {
        getValue("pipeline").setNumber(number);
    }

    /**
     * Retrieves an entry from the Limelight NetworkTable.
     *
     * @param key key for entry
     * @return the value of the given entry
     */
    private static NetworkTableEntry getValue(String key) {
        if (table == null) {
            table = NetworkTableInstance.getDefault();
        }
        return table.getTable("limelight").getEntry(key);
    }
}
