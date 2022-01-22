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
    private static RootNamespace rootNamespace = new RootNamespace("Limelight Values");
    private static NetworkTableInstance table;

    public Limelight() {
        rootNamespace.putBoolean("Is on target", this::isOnTarget);
        rootNamespace.putNumber("Horizontal offset from target", this::getHorizontalOffsetFromTarget);
        rootNamespace.putNumber("Vertical offset from target", this::getVerticalOffsetFromTarget);
        rootNamespace.putNumber("Target screen fill percent", this::getTargetAreaPercentage);
        rootNamespace.putNumber("Pipeline latency", this::getTargetLatency);
        rootNamespace.putNumber("Distance from target", this::calculateDistance);
    }

    /**
     * @return the distance between the limelight and the target using a (pre-calculated formula with a graph)
     */
    private double calculateDistance() {
        double x = getTargetWidthInPixels();
        return 37.905 * Math.pow(x, -0.977) * 0.05 / getTargetWidthInPixels();
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
    public double getHorizontalOffsetFromTarget() {
        return getValue("tx").getDouble(0.00);
    }

    /**
     * @return the vertical offset from crosshair to target (-20.5 degrees to 20.5 degrees)
     */
    public double getVerticalOffsetFromTarget() {
        return getValue("ty").getDouble(0.00);
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
     * sets pipeline number (0-9 value)
     *
     * @param number pipeline number (0-9)
     */
    public void setPipeline(int number) {
        getValue("pipeline").setNumber(number);
    }

    public void periodic() {
        rootNamespace.update();
    }

    /**
     * retrieve an entry from the Limelight NetworkTable.
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
