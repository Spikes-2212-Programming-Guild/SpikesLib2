package com.spikes2212.path;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

public class Path {
    private List<Waypoint> points;
    public Path(Waypoint... points) {
        this.points = new LinkedList<>(Arrays.asList(points));
    }

    public void fill(int middlePoints) {
        for (int i = 0; i < points.size() - 1; i++) {
            double xOffset = (points.get(i+1).getX() - points.get(i).getX()) / (middlePoints + 1);
            double yOffset = (points.get(i+1).getY() - points.get(i).getY()) / (middlePoints + 1);
            double tempX = points.get(i).getX() + xOffset, tempY = points.get(i).getY() + yOffset;
            while (tempX < points.get(i+1).getX()) {
                points.add(i, new Waypoint(tempX, tempY, points.get(i+1).getAngle()));
                i++;
                tempX += xOffset;
                tempY += yOffset;
            }
        }
    }

    public void smooth(double data_weight, double smooth_weight, double tolerance) {
        double [][] path = new double[points.size()][2];
        double [][] ogPath = Arrays.copyOf(path, path.length);
        for (int i = 0; i < points.size(); i++) {
            path[i] = points.get(i).toArray();
        }
        double change = tolerance;
        while (change >= tolerance) {
            change = 0;
            for(int i = 1; i < path.length - 1; i++) {
                for(int j = 0; j < path[i].length; j++) {
                    double aux = path[i][j];
                    path[i][j] += data_weight * (ogPath[i][j] - path[i][j])
                            + smooth_weight * (path[i-1][j] + path[i+1][j] - 2 * path[i][j]);
                    change = Math.abs(aux - path[i][j]);
                }
            }
        }

        for (int i = 0; i < path.length; i++) {
            points.set(i, new Waypoint(path[i][0], path[i][1], points.get(i).getAngle()));
        }
    }
}
