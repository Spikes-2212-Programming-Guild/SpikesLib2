package com.spikes2212.util;

import com.spikes2212.dashboard.Namespace;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.function.Supplier;

public class Limelight {

    private int limelightFocus;

    private static RootNamespace rootNamespace = new RootNamespace("Limelight Values");
    private static Namespace ConstantNamespace = rootNamespace.addChild("Constants");
    private static NetworkTableInstance table;

    private static Supplier<Double> angle = ConstantNamespace.addConstantDouble("Limelight Angle", 0);
    private static Supplier<Double> targetWidth = ConstantNamespace.addConstantDouble("Target Width", 0);
    private static Supplier<Double> cameraHeight = ConstantNamespace.addConstantDouble("Camera Height", 0);
    private static Supplier<Double> resolutionHeight = ConstantNamespace.addConstantDouble("Camera resolution height", 240);
    private static Supplier<Double> resolutionWidth = ConstantNamespace.addConstantDouble("Camera resolution width", 320);
    private static Supplier<Double> distance = ConstantNamespace.addConstantDouble("Target distance", 1.8);


    public Limelight(int focusLength) {
        limelightFocus = focusLength;
        rootNamespace.putBoolean("Is on target", this::isOnTarget);
        rootNamespace.putNumber("Limelight X", this::getHorizontalOffsetFromTarget);
        rootNamespace.putNumber("Limelight Y", this::getVerticalOffsetFromTarget);
        rootNamespace.putNumber("Limelight Area", this::getTargetAreaPercentage);
        rootNamespace.putNumber("Limelight Target Latency", this::getTargetLatency);
        rootNamespace.putNumber("Distance from target", this::calculateDistance);
        rootNamespace.putNumber("Target Angle", this::getVerticalOffsetFromTarget);
        rootNamespace.putNumber("Calculated Target Height", this::calculateTargetHeight);
        rootNamespace.putNumber("Object focus", this::getFocus);
        rootNamespace.putNumber("Get distance with focus", this::getDistanceWithFocus);
    }

    /**
     * Calculates the distance from the target.
     *
     * @return the distance from the target
     */
    public double calculateDistance() {
        double targetHeight = calculateTargetHeight();
        double targetAngle = getVerticalOffsetFromTarget();
        return Math.abs((targetHeight / 2 - cameraHeight.get()) / (Math.tan((targetAngle + angle.get()) *
                (Math.PI / 180))));
    }

    /**
     * Get target height in proportion to the width.
     *
     * @return target height in meters
     */
    private double calculateTargetHeight() {
        double pixelsPerMeterWidth = getTargetWidthInPixels() / targetWidth.get();
        double pixelsPerMeterHeight = pixelsPerMeterWidth * resolutionHeight.get() / resolutionWidth.get();
        return getTargetHeightInPixels() / pixelsPerMeterHeight;
    }

    /**
     * Uses a set distance that is given via the network table to calculate the focus of the limelight.
     *
     * @return limelight focus length (in pixels).
     */
    private double getFocus() {
        double objectWidth = targetWidth.get();
        double objectDistance = distance.get();
        double objectWidthInPixelsT = getTargetWidthInPixels();
        return (objectWidthInPixelsT * objectDistance) / objectWidth;
    }

    /**
     * Uses the autofocus properties of the limelight to calculate its distance from the target.
     *
     * @return limelight distance to target (in meters).
     */
    private double getDistanceWithFocus() {
        double objectWidth = targetWidth.get();
        double objectWidthInPixels = getTargetWidthInPixels();
        return (limelightFocus * objectWidth) / objectWidthInPixels;
    }

    /**
     * Gets whether a target is detected by the Limelight.
     *
     * @return true if a target is detected, false otherwise.
     */
    public boolean isOnTarget() {
        return getValue("tv").getDouble(0) == 1;
    }

    /**
     * Horizontal offset from crosshair to target (-27 degrees to 27 degrees).
     *
     * @return tx as reported by the Limelight.
     */
    public double getHorizontalOffsetFromTarget() {
        return getValue("tx").getDouble(0.00);
    }

    /**
     * Vertical offset from crosshair to target (-20.5 degrees to 20.5 degrees).
     *
     * @return ty as reported by the Limelight.
     */
    public double getVerticalOffsetFromTarget() {
        return getValue("ty").getDouble(0.00);
    }

    /**
     * Area that the detected target takes up in total camera FOV (0% to 100%).
     *
     * @return area of target.
     */
    public double getTargetAreaPercentage() {
        return getValue("ta").getDouble(0.00);
    }

    /**
     * Gets target skew or rotation (-90 degrees to 0 degrees).
     *
     * @return target skew.
     */
    public double getTargetSkew() {
        return getValue("ts").getDouble(0.00);
    }

    /**
     * Gets target latency (ms).
     *
     * @return target latency.
     */
    public double getTargetLatency() {
        return getValue("tl").getDouble(0.00);
    }

    /**
     * Gets the target width in pixels (0 pixels to 320 pixels).
     *
     * @return target width (pixels)
     */
    public double getTargetWidthInPixels() {
        return getValue("thor").getDouble(0.00);
    }

    /**
     * Gets the target width in pixels (0 pixels to 320 pixels).
     *
     * @return target height (pixels)
     */
    public double getTargetHeightInPixels() {
        return getValue("tvert").getDouble(0.00);
    }

    /**
     * Sets pipeline number (0-9 value).
     *
     * @param number pipeline number (0-9).
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
     * @param key key for entry.
     * @return NetworkTableEntry of given entry.
     */
    private static NetworkTableEntry getValue(String key) {
        if (table == null) {
            table = NetworkTableInstance.getDefault();
        }
        return table.getTable("limelight").getEntry(key);
    }
}
