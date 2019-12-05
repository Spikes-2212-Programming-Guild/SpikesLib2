package com.spikes2212.path;

public class Waypoint {
    private final double x, y, angle;
    private double v, d, curvature;
    public Waypoint(double x, double y, double angle) {
        this.x = x;
        this.y = y;
        this.angle = angle;
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

    public void setV(double v) {
        this.v = v;
    }

    public double getAngle() {
        return angle;
    }

    public void setD(double distance) {
        this.d = distance;
    }

    public double getD(){
        return d;
    }

    public double[] toArray() { return new double[]{x, y}; }

    public double getCurvature() {
        return curvature;
    }

    public void setCurvature(double curvature) {
        this.curvature = curvature;
    }

    public double distance(Waypoint point) {
        return Math.sqrt((x - point.getX())*(getX() - point.getX()) + (getY() - point.getY())*(getY() - point.getY()));
    }

    @Override
    public String toString() {
        return "x: " + x + " y: " + y + " angle: " + angle;
    }
}