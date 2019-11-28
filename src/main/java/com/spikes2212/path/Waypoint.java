package com.spikes2212.path;

public class Waypoint {
    private final double x, y, angle, v;
    private double d;
    public Waypoint(double x, double y, double angle, double v) {
        this.x = x;
        this.y = y;
        this.angle = angle;
        this.v = v;
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

    public double getAngle() {
        return angle;
    }

    public void setD(double distance) {
        this.d = distance;
    }

    public double getD(){
        return d;
    }

    public double[] toArray() {
        return new double[]{x, y, angle, v};
    }
}