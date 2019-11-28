package com.spikes2212.path;

public class Waypoint {
    private double x, y, angle, v, d;
    public Waypoint(double x, double y, double angle, double v) {
        this.x = x;
        this.y = y;
        this.angle = angle;
        this.v = v;
    }
    
    public void setD(double distance) {
        this.d = distance;
    }

    public double[] toArray() {
        return new double[]{x, y, angle, v};
    }
}