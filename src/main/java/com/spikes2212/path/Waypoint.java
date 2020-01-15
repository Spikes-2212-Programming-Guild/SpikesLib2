package com.spikes2212.path;

import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * This class represents a point in 2d space, with additional fields used for path generation and following.
 *
 * @author T
 */
public class Waypoint {

    private Translation2d position;

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
        this.position = new Translation2d(x,y);
    }

    public double getX() {
        return position.getX();
    }

    public double getY() {
        return position.getY();
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
    public double[] toArray() { return new double[]{getX(), getY()}; }

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
        return Math.sqrt((getX() - point.getX())*(getX() - point.getX()) + (getY() - point.getY())*(getY() - point.getY()));
    }

    @Override
    public String toString() {
        return "x: " + getX() + " y: " + getY();
    }

    @Override
    public boolean equals(Object point) {
        if(!(point instanceof Waypoint)) return false;
        Waypoint other = (Waypoint) point;
        return getX() == other.getX() && getY() == other.getY();
    }
}