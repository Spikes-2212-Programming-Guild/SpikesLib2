package com.spikes2212.path;

import java.util.Arrays;
import java.util.List;

public class Path {
    private List<Waypoint> points;
    public Path(Waypoint... points) {
        this.points = Arrays.asList(points);
    }

    public void fill(int middlePoints) {
        for (int i = 0; i < points.size(); i++) {
            double xOffset = (points.get(i+1).getX() - points.get(i).getX()) / (middlePoints + 1);
            double yOffset = (points.get(i+1).getY() - points.get(i).getY()) / (middlePoints + 1);
            double tempX = points.get(i).getX() + xOffset, tempY = points.get(i).getY() + yOffset;
            while (tempX != points.get(i+1).getX()) {
                points.add(i, new Waypoint(tempX, tempY, points.get(i+1).getAngle()));
                i++;
                tempX += xOffset;
                tempY += yOffset;
            }
        }
    }
}