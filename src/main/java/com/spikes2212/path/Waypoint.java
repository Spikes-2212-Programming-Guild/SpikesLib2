package com.spikes2212.path;

/**
 * This class represents a point in 2d space, with additional fields used for path generation and following.
 *
 * @author T
 */
public class Waypoint {

    /**
     * The x coordinate.
     */
    private final double x;

    /**
     * The y coordinate.
     */
    private final double y;

    /**
     * The velocity at the given point (used for path following).
     */
    private double v;

    /**
     * The distance from the origin of the path along the path.
     */
    private double d;

    /**
     * The curvature of the path at the point.
     */
    private double curvature;

    public Waypoint(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getV() {
        return v;
    }

    void setV(double v) {
        this.v = v;
    }

    void setD(double distance) {
        this.d = distance;
    }

    public double getD(){
        return d;
    }

    /**
     * returns an array of the point's coordinates
     * @return an array of the point's coordinates
     */
    public double[] toArray() { return new double[]{x, y}; }

    public double getCurvature() {
        return curvature;
    }

    void setCurvature(double curvature) {
        this.curvature = curvature;
    }

    /**
     * returns the distance from the given point to {@code this}.
     * @param point the point to calculate the distance from
     * @return the distance between the points
     */
    public double distance(Waypoint point) {
        return Math.sqrt((x - point.getX())*(getX() - point.getX()) + (getY() - point.getY())*(getY() - point.getY()));
    }

    @Override
    public String toString() {
        return "x: " + x + " y: " + y;
    }
}