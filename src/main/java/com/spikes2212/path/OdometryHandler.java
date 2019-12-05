package com.spikes2212.path;

import java.awt.geom.Point2D;
import java.util.function.Supplier;

/**
 *<ul><i><a><h1><li><strong><b>The calculate method should be called repeatedly, otherwise your robot's position
 * 	would not be recalculated, and therefore will be substantially inaccurate.
 * </b></strong></li></h1></a></i></ul>
 *
 * a class which uses encoders to find the progress of a robot between discrete
 * times
 *
 * @author noam mantin
 *
 */
public class OdometryHandler {

    private RelativeDataSupplier yawDiff, leftDistance, rightDistance;
    private Supplier<Double> angleSupplier;
    private double x, y;

    /**
     * creates a new {@link OdometryHandler} object, with given parameters
     * @param leftPosition  the left position supplier
     * @param rightPosition the right position supplier
     * @param angleSupplier the angle supplier
     * @param x the starting x coordinate of your robot
     * @param y the starting y coordinate of your robot
     * @param baseYaw the angle between the x axis and your robot's starting position
     */
    public OdometryHandler(Supplier<Double> leftPosition, Supplier<Double> rightPosition, Supplier<Double> angleSupplier
            , double x, double y, double baseYaw) {
        this.leftDistance = new RelativeDataSupplier(leftPosition);
        this.rightDistance = new RelativeDataSupplier(rightPosition);
        this.yawDiff = new RelativeDataSupplier(angleSupplier);
        this.angleSupplier = angleSupplier;
        this.x = x;
        this.y = y;
    }

    /**
     * <ul><i><a><h1><li><strong><b>This method should be called repeatedly, otherwise your robot's position
     * 	would not be recalculated, and therefore will be substantially inaccurate.
     * </b></strong></li></h1></a></i></ul>
     *
     * Calculates and returns the displacement of the robot since the last
     * check.</br>
     *
     * for more information about this process, see pages 2910-2911 in the main
     * article from the README file
     *
     * @return the position rate vector of the robot (ΔX and ΔY), from the
     *         last call to this method, in the navigation frame.
     *
     */
    public void calculate() {
        // getting the yaw angle of the robot in the end of the movement
        double yaw = Math.toRadians(angleSupplier.get());
        /*
         * The robot's displacement's norm, i.e the length of the straight line
         * starting from the previous center location of the robot and ending in
         * the current center location. Signified in the article by the letter
         * Δλ
         */
        double centerDistance;

        /*
         * Getting the distance each side of the robot has passed. Signified in
         * the article by the letters a(l) and a(r)
         */
        double rightDistance = this.rightDistance.get();
        double leftDistance = this.leftDistance.get();

        /*
         * The yaw difference .Signified by Δϕ in the article. (equation 22)
         */
        double yawDifference = yawDiff.get();
        /*
         * calculating the length of the arch the middle of the robot has
         * passed. Signified in the article as ak (equation 21)
         */
        double archDistance = (rightDistance + leftDistance) / 2;

        if (yawDifference == 0) {
            // the robot moved in a straight line
            centerDistance = Math.abs(rightDistance);
        } else {
            /*
             * calculating the rotation radius of the robot's movement.
             * Signified by r in the article. (equation 23).
             */
            double rotationRadius = archDistance / yawDifference;
            /*
             * using the cosine law to find the robot's displacement's norm
             * (Δλ). (equation 24)
             */
            centerDistance = 2 * Math.sin(yawDifference / 2) * rotationRadius;
        }

        // finding the argument of the displacement vector.
        double arg = yaw - 1 / 2.0 * yawDifference;

        // if turned to the opposite direction
        if (archDistance < 0) {
            arg += Math.PI;
        }
        /*
         * Building the cartesian displacement vector by an argument (arg) and a
         * norm (Δλ).
         *
         * Now, as we have the argument and the norm of the displacement vector
         * (or in other words, the polar form of the vector), we can convert it
         * to the Cartesian form (represented by its X and Y components). Doing
         * so gives us the robot's position in the navigation system (equation
         * 26)
         */
        Point2D point = convertPolarToCartesian(centerDistance, -arg);
        x += point.getX();
        y += point.getY();
    }

    /**
     * returns the robot's current x coordinate
     * @return the robot's current x coordinate
     */
    public double getX() {
        return x;
    }

    /**
     * returns the robot's current y coordinate
     * @return the robot's current y coordinate
     */
    public double getY() {
        return y;
    }

    /**
     * returns the robot's current angle
     * @return the robot's current angle
     */
    public double getYaw() {
        return angleSupplier.get();
    }


    public Waypoint getWaypoint() {
        return new Waypoint(getX(), getY(), getYaw());
    }

    /**
     * sets the robot's coordinates
     * @param x the x coordinate
     * @param y the y coordinate
     */
    public void set(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * This method receives a vector presented in the polar form (norm and
     * argument) and return the cartesian form of this vector (the X component
     * and the Y component)
     *
     * @param norm
     *            the vector's length
     * @param argument
     *            the argument of the vector (the argument between the vector
     *            and the X axis).
     * @return a point representing the vector by its X and Y component (the
     *         cartesian form of the vector) in that order: (X,Y)
     */
    public static Point2D convertPolarToCartesian(double norm, double argument) {
        // calculating the X and Y components
        double legX = norm * Math.cos(argument);
        double legY = norm * Math.sin(argument);
        // returning the result as a point
        return new Point2D.Double(legX, legY);
    }
}