package com.spikes2212.path;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;

import java.util.function.Supplier;

/**
 *<ul><i><a><h1><li><strong><b>The calculate method should be called repeatedly, otherwise your robot's position
 * 	would not be recalculated, and therefore will be substantially inaccurate.
 * </b></strong></li></h1></a></i></ul>
 *
 * a class which uses encoders to find the progress of a robot between discrete
 * times
 */
public class OdometryHandler {

    private Supplier<Double> leftPosition, rightPosition;
    private Supplier<Double> yaw;
    private double lastLeftPosition = 0, lastRightPosition = 0, lastYaw = 0;
    private Pose2d pose;

    /**
     * creates a new {@link OdometryHandler} object, with given parameters
     * @param leftPosition  the left position supplier
     * @param rightPosition the right position supplier
     * @param angleSupplier the angle supplier
     * @param x the initial x coordinate
     * @param y the initial y coordinate
     */
    public OdometryHandler(Supplier<Double> leftPosition, Supplier<Double> rightPosition, Supplier<Double> angleSupplier
            , double x, double y) {
        this.leftPosition = leftPosition;
        this.rightPosition = rightPosition;
        this.yaw = angleSupplier;
        pose = new Pose2d(new Translation2d(x, y), Rotation2d.fromDegrees(yaw.get()));
    }

    public void calculate() {
        double leftPosition = this.leftPosition.get();
        double rightPosition = this.rightPosition.get();
        double yaw = this.yaw.get();
        double deltaLeftDistance = leftPosition - lastLeftPosition;
        double deltaRightDistance = rightPosition - lastRightPosition;
        double deltaYaw = yaw - lastYaw;

        double averageDeltaDistance = (deltaLeftDistance + deltaRightDistance) / 2.0;
        Rotation2d angle = Rotation2d.fromDegrees(deltaYaw);

        lastYaw = yaw;
        lastLeftPosition = leftPosition;
        lastRightPosition = rightPosition;


        Pose2d newPose = pose.exp(
                new Twist2d(averageDeltaDistance, 0.0, angle.getRadians()));

        pose = new Pose2d(newPose.getTranslation(), Rotation2d.fromDegrees(yaw));
    }
    /**
     * returns the robot's current x coordinate
     * @return the robot's current x coordinate
     */
    public double getX() {
        return pose.getTranslation().getX();
    }

    /**
     * returns the robot's current y coordinate
     * @return the robot's current y coordinate
     */
    public double getY() {
        return pose.getTranslation().getY();
    }

    /**
     * returns the robot's current angle
     * @return the robot's current angle
     */
    public double getYaw() {
        return yaw.get();
    }

    /**
     * returns the robot's coordinates as a {@link Waypoint} instance
     * @return robot's coordinates as a {@link Waypoint} instance
     */
    public Waypoint getWaypoint() {
        return new Waypoint(getY(), getX());
    }

    public void set(double x, double y) {
        this.pose = new Pose2d(new Translation2d(x, y), new Rotation2d(yaw.get()));
        this.lastLeftPosition = 0;
        this.lastRightPosition = 0;
    }
}