package com.spikes2212.util;

import com.spikes2212.dashboard.Namespace;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.function.Supplier;

/**
 * This class is a limelight wrapper that includes a function that calculates the distance of the limelight from a target in meters.
 * This class assumes you are using the 960x720 processing resolution on the limelight.
 *
 * @author Yotam Yizhar
 */

public class Limelight {

    private static RootNamespace rootNamespace = new RootNamespace("Limelight Values");
    private static Namespace ConstantNamespace = rootNamespace.addChild("Constants");
    private static NetworkTableInstance table;
    private static final double FOCAL_LENGTH = 425; // The focal length, the distance between the lens and the sensor of the limelight (in px).

    private static Supplier<Double> targetWidth = ConstantNamespace.addConstantDouble("Target Width", 0);


    public Limelight() {
        rootNamespace.putBoolean("Is on target", this::isOnTarget);
        rootNamespace.putNumber("Horizontal offset from target", this::getHorizontalOffsetFromTarget);
        rootNamespace.putNumber("Vertical offset from target", this::getVerticalOffsetFromTarget);
        rootNamespace.putNumber("Target screen fill percent", this::getTargetAreaPercentage);
        rootNamespace.putNumber("Pipeline latency", this::getTargetLatency);
        rootNamespace.putNumber("Distance from target", this::calculateDistance);
    }

    /**
     * @return the distance between the limelight and the target (in meters)
     */
    private double calculateDistance() {
        double objectWidth = targetWidth.get();
        double objectWidthInPixels = getTargetWidthInPixels();
        return (FOCAL_LENGTH * objectWidth) / objectWidthInPixels;
    }

    /**
     * @return whether a target is detected by the limelight
     */
    public boolean isOnTarget() {
        return getValue("tv").getDouble(0) == 1;
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
     * @return the target width in pixels (0 pixels to 720 pixels).
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

    public void periodic() {
        rootNamespace.update();
    }

    /**
     * Helper method to get an entry from the Limelight NetworkTable.
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
