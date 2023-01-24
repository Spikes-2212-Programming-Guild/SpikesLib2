package com.spikes2212.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * This class is a limelight wrapper.<br>
 * <b>This class assumes you are using the 960x720 processing resolution on the limelight.</b>
 *
 * @author Yotam Yizhar
 */
public class Limelight {

    private Translation3d translation3d;
    private Rotation3d rotation3d;
    private Pose3d pose3d;

    protected static NetworkTableInstance table;

    public Limelight() {
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

    @Deprecated(since = "2023", forRemoval = true)
    public void periodic() {
    }

    /**
     * @deprecated use {@link #hasTarget()}
     */
    @Deprecated(since = "2023", forRemoval = true)
    public boolean isOnTarget() {
        return this.hasTarget();
    }

    /**
     * @return whether a target is detected by the limelight
     */
    public boolean hasTarget(){
        return getValue("tv").getDouble(0) == 1;
    }

    /**
     * @return the current limelight's pipeline
     */
    public int getPipeline() {
        return (int) getValue("getpipe").getNumber(0);
    }

    /**
     * @return the robot's current pose
     */
    public Pose3d getRobotPose() {
        double[] result = getValue("botpose").getDoubleArray(new double[]{});
        translation3d = new Translation3d(result[0], result[1], result[2]);
        rotation3d = new Rotation3d(result[3], result[4], result[5]);
        pose3d = new Pose3d(translation3d, rotation3d);
        return pose3d;
    }

    /**
     * Transforms camera into 3D
     *
     * @return the 3D value
     */
    public Transform3d getCameraTransformation() {
        double[] result = getValue("camtran").getDoubleArray(new double[] {});
        translation3d = new Translation3d(result[0], result[1], result[2]);
        rotation3d = new Rotation3d(result[5], result[4], result[3]);
        return new Transform3d(translation3d, rotation3d);
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
     * @return the horizontal offset from crosshair to target (-27 degrees to 27 degrees)
     */
    public double getHorizontalOffsetFromTargetInDegrees() {
        return getValue("tx").getDouble(0.00);
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
}
