package com.spikes2212.path;

import edu.wpi.first.wpilibj.Filesystem;

import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.*;

public class Path {

    /**
     * @param spacing         the distance between two points filled in between points given as parameters
     * @param smoothWeight    how smooth to make the path, should be about 0.75 to 0.98
     * @param tolerance       the smoothing tolerance
     * @param maxVelocity     the robot's maximum velocity
     * @param turningConstant speed constant at curves (the higher it is, the faster you turn)
     * @param maxAcceleration the robot's maximum acceleration
     * @param points          the initial points on the path. Apart from the edges, non of the points are guaranteed
     *                        to be on the final path
     * @return the generated path
     */
    public static List<Waypoint> generate(List<Waypoint> points, double spacing, double smoothWeight, double tolerance,
                                          double maxVelocity, double turningConstant, double maxAcceleration) {
        fill(points, spacing);
        smooth(points, smoothWeight, tolerance);
        calculateDistances(points);
        calculateCurvatures(points);
        calculateMaxVelocities(points, maxVelocity, turningConstant);
        smoothVelocities(points, maxAcceleration);
        return points;
    }

    private static void fill(List<Waypoint> points, double spacing) {
        for(int i = 0; i < points.size() - 1; i++) {
            Waypoint startPoint = points.get(i);
            double length = points.get(i).distance(points.get(i + 1));
            int pointsThatFit = (int)(length / spacing);
            Waypoint vector = new Waypoint((points.get(i + 1).getX() - points.get(i).getX()) * (spacing / length),
                    (points.get(i + 1).getY() - points.get(i).getY()) * (spacing / length));
            for(int j = 0; j < pointsThatFit; j++, i++) {
                points.add(i + 1, new Waypoint(
                        startPoint.getX() + vector.getX() * (j + 1),
                        startPoint.getY() + vector.getY() * (j + 1)
                ));
            }
        }
    }

    private static void smooth(List<Waypoint> points, double smoothWeight, double tolerance) {
        double dataWeight = 1 - smoothWeight;
        double[][] path = new double[points.size()][2];
        for(int i = 0; i < points.size(); i++) {
            path[i] = points.get(i).toArray();
        }
        double[][] ogPath = Arrays.copyOf(path, path.length);
        double change = tolerance;
        while(change >= tolerance) {
            change = 0;
            for(int i = 1; i < path.length - 1; i++) {
                for(int j = 0; j < path[i].length; j++) {
                    double aux = path[i][j];
                    path[i][j] += dataWeight * (ogPath[i][j] - path[i][j])
                            + smoothWeight * (path[i -1][j] + path[i+1][j] - 2 * path[i][j]);
                    change += Math.abs(aux - path[i][j]);
                }
            }
        }

        for (int i = 0; i < path.length; i++) {
            points.set(i, new Waypoint(path[i][0], path[i][1]));
        }
    }

    private static void calculateDistances(List<Waypoint> points) {
        double previousDistance = 0;
        points.get(0).setD(0);
        for(int i = 1; i < points.size(); i++) {
            previousDistance += points.get(i).distance(points.get(i - 1));
            points.get(i).setD(previousDistance);
        }
    }

    private static void calculateCurvatures(List<Waypoint> points) {
        for(int i = 1; i < points.size() - 1; i++) {
            double x1 = points.get(i).getX();
            double y1 = points.get(i).getY();
            double x2 = points.get(i - 1).getX();
            double y2 = points.get(i - 1).getY();
            double x3 = points.get(i + 1).getX();
            double y3 = points.get(i + 1).getY();
            if(x1 == x2) x2 += 0.000001;
            double k1 = 0.5 * (x1 * x1 + y1 * y1 - x2 * x2 - y2 * y2) / (x1 - x2);
            double k2 = (y1 - y2) / (x1 - x2);
            double b = 0.5 * (x2 * x2 - 2 * x2 * k1 + y2 * y2 - x3 * x3 + 2 * x3 * k1 - y3 * y3) / (x3 * k2 - y3 + y2 - x2 * k2);
            double a = k1 - k2 * b;
            double r = Math.sqrt((x1 - a) * (x1 - a) + (y1 - b) * (y1 - b));
            points.get(i).setCurvature(1 / r);
        }
    }

    private static void calculateMaxVelocities(List<Waypoint> points, double maxVelocity, double turningConstant) {
        for(Waypoint p : points) {
            p.setV(Math.min(maxVelocity, turningConstant / p.getCurvature()));
        }
    }

    private static void smoothVelocities(List<Waypoint> points, double maxAcceleration) {
        points.get(points.size() - 1).setV(0);
        for(int i = points.size() - 2; i >= 0; i--) {
            double distance = points.get(i).distance(points.get(i + 1));
            points.get(i).setV(Math.min(points.get(i).getV(),
                    Math.sqrt(Math.pow(points.get(i + 1).getV(), 2) + 2 * maxAcceleration * distance)));
        }
    }

    /**
     * Exports the path to a CSV file with the following format:
     * x,y,velocity,distance,curvature.
     *
     * @param path the CSV file
     */
    public static void exportToCSV(List<Waypoint> points, java.nio.file.Path path) {
        try(BufferedWriter writer = Files.newBufferedWriter(path, StandardCharsets.US_ASCII)) {
            StringBuilder s = new StringBuilder("x,y,velocity,distance,curvature\n");
            for(Waypoint w : points) {
                s.append(w.getX()).append(",").append(w.getY()).append(",").append(w.getV()).append(",")
                        .append(w.getD()).append(",").append(w.getCurvature()).append("\n");
            }
            writer.write(s.toString(), 0, s.length());
        } catch(IOException ioe) {
            ioe.printStackTrace();
        }
    }

    public static List<Waypoint> loadFromCSV(java.nio.file.Path path) {
        List<Waypoint> waypoints = new ArrayList<>();
        try {
            List<String> lines = Files.readAllLines(path);
            lines.remove(0);
            for(String line : lines) {
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
        return waypoints;
    }

    public static List<Waypoint> loadFromCSV(String name) {
        return loadFromCSV(Paths.get(Filesystem.getDeployDirectory().toString(), name));
    }
}
