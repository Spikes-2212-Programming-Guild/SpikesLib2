package com.spikes2212.path;

import edu.wpi.first.wpilibj.Filesystem;

import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;

public class Paths {

    /**
     * @param spacing         the distance between two path filled in between path given as parameters
     * @param smoothWeight    how smooth to make the path, should be about 0.75 to 0.98
     * @param tolerance       the smoothing tolerance
     * @param maxVelocity     the robot's maximum velocity
     * @param turningConstant speed constant at curves (the higher it is, the faster you turn)
     * @param maxAcceleration the robot's maximum acceleration
     * @param path            the initial points on the path. Apart from the edges, non of the points are guaranteed
     *                        to be on the final path
     * @return the generated path
     */
    public static List<Waypoint> generate(List<Waypoint> path, double spacing, double smoothWeight, double tolerance,
                                          double maxVelocity, double turningConstant, double maxAcceleration) {
        List<Waypoint> points = new ArrayList<>(path);
        fill(points, spacing);
        smooth(points, smoothWeight, tolerance);
        calculateDistances(points);
        calculateCurvatures(points);
        calculateMaxVelocities(points, maxVelocity, turningConstant);
        smoothVelocities(points, maxAcceleration);
        return points;
    }

    /**
     * @param spacing         the distance between two path filled in between path given as parameters
     * @param smoothWeight    how smooth to make the path, should be about 0.75 to 0.98
     * @param tolerance       the smoothing tolerance
     * @param maxVelocity     the robot's maximum velocity
     * @param turningConstant speed constant at curves (the higher it is, the faster you turn)
     * @param maxAcceleration the robot's maximum acceleration
     * @param path            the initial points on the path. Apart from the edges, non of the points are guaranteed
     *                        to be on the final path
     * @return the generated path
     */
    public static List<Waypoint> generate(double spacing, double smoothWeight, double tolerance, double maxVelocity,
                                          double turningConstant, double maxAcceleration, Waypoint... path) {
        return generate(Arrays.asList(path), spacing, smoothWeight, tolerance, maxVelocity, turningConstant,
                maxAcceleration);
    }

    private static void fill(List<Waypoint> path, double spacing) {
        for(int i = 0; i < path.size() - 1; i++) {
            Waypoint startPoint = path.get(i);
            double length = path.get(i).distance(path.get(i + 1));
            int pathThatFit = (int)(length / spacing);
            Waypoint vector = new Waypoint((path.get(i + 1).getX() - path.get(i).getX()) * (spacing / length),
                    (path.get(i + 1).getY() - path.get(i).getY()) * (spacing / length));
            for(int j = 0; j < pathThatFit; j++, i++) {
                path.add(i + 1, new Waypoint(
                        startPoint.getX() + vector.getX() * (j + 1),
                        startPoint.getY() + vector.getY() * (j + 1)
                ));
            }
        }
    }

    private static void smooth(List<Waypoint> path, double smoothWeight, double tolerance) {
        double dataWeight = 1 - smoothWeight;
        double[][] newPath = new double[path.size()][2];
        for(int i = 0; i < path.size(); i++) {
            newPath[i] = path.get(i).toArray();
        }
        double[][] ogPath = Arrays.copyOf(newPath, newPath.length);
        double change = tolerance;
        while(change >= tolerance) {
            change = 0;
            for(int i = 1; i < newPath.length - 1; i++) {
                for(int j = 0; j < newPath[i].length; j++) {
                    double aux = newPath[i][j];
                    newPath[i][j] += dataWeight * (ogPath[i][j] - newPath[i][j])
                            + smoothWeight * (newPath[i - 1][j] + newPath[i + 1][j] - 2 * newPath[i][j]);
                    change += Math.abs(aux - newPath[i][j]);
                }
            }
        }

        for(int i = 0; i < newPath.length; i++) {
            path.set(i, new Waypoint(newPath[i][0], newPath[i][1]));
        }
    }

    private static void calculateDistances(List<Waypoint> path) {
        double previousDistance = 0;
        path.get(0).setD(0);
        for(int i = 1; i < path.size(); i++) {
            previousDistance += path.get(i).distance(path.get(i - 1));
            path.get(i).setD(previousDistance);
        }
    }

    private static void calculateCurvatures(List<Waypoint> path) {
        for(int i = 1; i < path.size() - 1; i++) {
            double x1 = path.get(i).getX();
            double y1 = path.get(i).getY();
            double x2 = path.get(i - 1).getX();
            double y2 = path.get(i - 1).getY();
            double x3 = path.get(i + 1).getX();
            double y3 = path.get(i + 1).getY();
            if(x1 == x2) x2 += 0.000001;
            double k1 = 0.5 * (x1 * x1 + y1 * y1 - x2 * x2 - y2 * y2) / (x1 - x2);
            double k2 = (y1 - y2) / (x1 - x2);
            double b = 0.5 * (x2 * x2 - 2 * x2 * k1 + y2 * y2 - x3 * x3 + 2 * x3 * k1 - y3 * y3) / (x3 * k2 - y3 + y2 - x2 * k2);
            double a = k1 - k2 * b;
            double r = Math.sqrt((x1 - a) * (x1 - a) + (y1 - b) * (y1 - b));
            path.get(i).setCurvature(1 / r);
        }
    }

    private static void calculateMaxVelocities(List<Waypoint> path, double maxVelocity, double turningConstant) {
        for(Waypoint p : path) {
            p.setV(Math.min(maxVelocity, turningConstant / p.getCurvature()));
        }
    }

    private static void smoothVelocities(List<Waypoint> path, double maxAcceleration) {
        path.get(path.size() - 1).setV(0);
        for(int i = path.size() - 2; i >= 0; i--) {
            double distance = path.get(i).distance(path.get(i + 1));
            path.get(i).setV(Math.min(path.get(i).getV(),
                    Math.sqrt(Math.pow(path.get(i + 1).getV(), 2) + 2 * maxAcceleration * distance)));
        }
    }

    /**
     * Exports the path to a CSV file with the following format:
     * x,y,velocity,distance,curvature.
     *
     * @param path the path to export
     * @param file the CSV file
     */
    public static void exportToCSV(List<Waypoint> path, Path file) {
        try(BufferedWriter writer = Files.newBufferedWriter(file, StandardCharsets.US_ASCII)) {
            StringBuilder s = new StringBuilder("x,y,velocity,distance,curvature\n");
            for(Waypoint w : path) {
                s.append(w.getX()).append(",").append(w.getY()).append(",").append(w.getV()).append(",")
                        .append(w.getD()).append(",").append(w.getCurvature()).append("\n");
            }
            writer.write(s.toString(), 0, s.length());
        } catch(IOException ioe) {
            ioe.printStackTrace();
        }
    }

    /**
     * loads a path from the given csv file
     *
     * @param path the csv file to import from
     * @return the path
     */
    public static List<Waypoint> loadFromCSV(Path path) {
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
        return loadFromCSV(java.nio.file.Paths.get(Filesystem.getDeployDirectory().toString(), name));
    }
}
