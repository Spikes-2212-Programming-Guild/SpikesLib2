package com.spikes2212.path;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

/**
 * This class represents a path.
 *
 * @author T
 */
public class Path {

    /**
     * The points on the path.
     */
    private List<Waypoint> points;

    /**
     * Initializes a path.
     * @param middlePoints the amount of points to fill between points
     * @param data_weight should be 1 minus smooth_weight
     * @param smooth_weight how smooth to make the path, should be about 0.6 to 0.8
     * @param tolerance the smoothing tolerance
     * @param maxVelocity the robot's maximum velocity
     * @param turningConstant speed constant at curves (the higher it is, the faster you turn)
     * @param maxAcceleration the robot's maximum acceleration
     * @param points the initial points on the path. Apart from the edges, non of the points are guaranteed
     *               to be on the final path
     */
    public Path(int middlePoints, double data_weight, double smooth_weight, double tolerance,
                double maxVelocity, double turningConstant, double maxAcceleration, Waypoint... points) {
        this.points = new LinkedList<>(Arrays.asList(points));
        generate(middlePoints, data_weight, smooth_weight, tolerance, maxVelocity, turningConstant, maxAcceleration);
    }

    private Path(List<Waypoint> points) {
        this.points = points;
    }

    public List<Waypoint> getPoints() {
        return points;
    }

    private void generate(int middlePoints, double data_weight, double smooth_weight, double tolerance,
                          double maxVelocity, double k, double maxAcceleration) {
        fill(middlePoints);
        smooth(data_weight, smooth_weight, tolerance);
        calculateDistances();
        calculateCurvatures();
        calculateMaxVelocities(maxVelocity, k);
        smoothVelocities(maxAcceleration);
    }

    private void fill(double distance) {
        for (int i = 0; i < points.size() - 1; i++) {
            double slopeAngle = Math.atan2(points.get(i+1).getY() - points.get(i).getY(),
                        points.get(i+1).getX() - points.get(i).getX());
            double xOffset = distance * Math.cos(slopeAngle);
            double yOffset = distance * Math.sin(slopeAngle);
            double tempX = points.get(i).getX() + xOffset, tempY = points.get(i).getY() + yOffset;
            while (tempX < points.get(i+1).getX()) {
                points.add(i, new Waypoint(tempX, tempY));
                i++;
                tempX += xOffset;
                tempY += yOffset;
            }
        }
    }

    private void smooth(double data_weight, double smooth_weight, double tolerance) {
        double [][] path = new double[points.size()][2];
        for (int i = 0; i < points.size(); i++) {
            path[i] = points.get(i).toArray();
        }
        double [][] ogPath = Arrays.copyOf(path, path.length);
        double change = tolerance;
        while (change >= tolerance) {
            change = 0;
            for(int i = 1; i < path.length - 1; i++) {
                for(int j = 0; j < path[i].length; j++) {
                    double aux = path[i][j];
                    path[i][j] += data_weight * (ogPath[i][j] - path[i][j])
                            + smooth_weight * (path[i-1][j] + path[i+1][j] - 2 * path[i][j]);
                    change += Math.abs(aux - path[i][j]);
                }
            }
        }

        for (int i = 0; i < path.length; i++) {
            points.set(i, new Waypoint(path[i][0], path[i][1]));
        }
    }

    private void calculateDistances() {
        double previousDistance = 0;
        points.get(0).setD(0);
        for (int i = 1; i < points.size(); i++) {
            previousDistance += points.get(i).distance(points.get(i-1));
            points.get(i).setD(previousDistance);
        }
    }

    private void calculateCurvatures() {
        for(int i = 1; i < points.size() - 1; i++) {
            double x1 = points.get(i).getX();
            double y1 = points.get(i).getY();
            double x2 = points.get(i - 1).getX();
            double y2 = points.get(i - 1).getY();
            double x3 = points.get(i + 1).getX();
            double y3 = points.get(i + 1).getY();
            if(x1 == x2) x2 += 0.000001;
            double k1 = 0.5 * (x1*x1 + y1*y1 - x2*x2 - y2*y2) / (x1 - x2);
            double k2 = (y1 - y2) / (x1 - x2);
            double b = 0.5 * (x2*x2 - 2*2*k1 + y2*y2 - x3*x3 + 2*x3*k1 - y3*y3) / (x3*k2 - y3 + y2 - x2*k2);
            double a = k1 - k2 * b;
            double r = Math.sqrt((x1-a)*(x1-a) + (y1-b)*(y1-b));
            points.get(i).setCurvature(1/r);
        }
    }

    private void calculateMaxVelocities(double maxVelocity, double k) {
        for (Waypoint p : points) {
            p.setV(Math.max(maxVelocity, k/p.getCurvature()));
        }
    }

    private void smoothVelocities(double maxAcceleration) {
        points.get(points.size() - 1).setV(0);
        for (int i = points.size() - 2; i >= 0; i--) {
            double distance;
            distance = points.get(i).distance(points.get(i+1));
            points.get(i).setV(Math.min(points.get(i).getV(),
                    Math.sqrt(Math.pow(points.get(i+1).getV(), 2) + 2*maxAcceleration*distance)));
        }
    }

    /**
     * Exports the path to a CSV file with the following format:
     *  x,y,velocity,distance,curvature.
     * @param path the CSV file
     */
    public void exportToCSV(java.nio.file.Path path) {
        try (BufferedWriter writer = Files.newBufferedWriter(path, Charset.forName("US-ASCII"))) {
            String s = "x,y,velocity,distance,curvature\n";
            for (Waypoint w : getPoints()) {
                s += w.getX() + "," + w.getY() + "," + w.getV() + "," + w.getD() + ","
                        + w.getCurvature() + "\n";
            }
        } catch (IOException ioe) {
            ioe.printStackTrace();
        }
    }

    public Path importFromCSV(java.nio.file.Path path) {
        List<Waypoint> waypoints = new LinkedList<>();
        try {
            List<String> lines = Files.readAllLines(path);
            for (String line : lines) {
                String[] values = line.split(",");
                Waypoint point = new Waypoint(Double.parseDouble(values[0]),
                        Double.parseDouble(values[1]));
                point.setV(Double.parseDouble(values[2]));
                point.setD(Double.parseDouble(values[3]));
                point.setCurvature(Double.parseDouble(values[4]));
                waypoints.add(point);
            }
        } catch (IOException ioe) {
            ioe.printStackTrace();
        }
        return new Path(waypoints);
    }
}
