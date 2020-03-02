package com.spikes2212.path;

import java.util.List;

/**
 * This class represents a PurePursuitController.
 * You should the getSpeeds method periodically.
 *
 * @author T
 */
public class PurePursuitController {
    private OdometryHandler odometryHandler;
    private List<Waypoint> path;
    private int lastClosestIndex = 0, lastLookaheadIndex = 0;
    private double lookaheadDistance;
    private double robotWidth;

    private RateLimiter rateLimiter;

    public PurePursuitController(OdometryHandler odometryHandler, List<Waypoint> path, double lookaheadDistance,
                                 double maxRate, double robotWidth, double period) {
        this.odometryHandler = odometryHandler;
        this.path = path;
        this.lookaheadDistance = lookaheadDistance;
        this.robotWidth = robotWidth;
        this.rateLimiter = new RateLimiter(maxRate, period);
    }

    public PurePursuitController(OdometryHandler odometryHandler, List<Waypoint> path, double lookaheadDistance,
                                 double maxRate, double robotWidth) {
        this(odometryHandler, path, lookaheadDistance, maxRate, robotWidth, 0.02);
    }

    public OdometryHandler getOdometryHandler() {
        return odometryHandler;
    }

    public void setOdometryHandler(OdometryHandler odometryHandler) {
        this.odometryHandler = odometryHandler;
    }

    public List<Waypoint> getPath() {
        return path;
    }

    public void setPath(List<Waypoint> path) {
        this.path = path;
    }

    public double getLookaheadDistance() {
        return lookaheadDistance;
    }

    public void setLookaheadDistance(double lookaheadDistance) {
        this.lookaheadDistance = lookaheadDistance;
    }

    private Waypoint closestPoint() {
        Waypoint robot = odometryHandler.getWaypoint();
        double minDistance = Double.POSITIVE_INFINITY, distance;
        int minIndex = lastClosestIndex;
        for(int i = lastClosestIndex; i < path.size(); i++) {
            if((distance = path.get(i).distance(robot)) < minDistance) {
                minIndex = i;
                minDistance = distance;
            }
        }
        lastClosestIndex = minIndex;
        return path.get(minIndex);
    }

    private Waypoint getLookaheadPoint() {
        Waypoint robot = odometryHandler.getWaypoint();
        for(int i = lastLookaheadIndex; i < path.size() - 1; i++) {
            Waypoint segment = new Waypoint(path.get(i + 1).getX() - path.get(i).getX()
                    , path.get(i + 1).getY() - path.get(i).getY());
            Waypoint robotToStart = new Waypoint(path.get(i).getX() - robot.getX()
                    , path.get(i).getY() - robot.getY());
            double a = segment.getX() * segment.getX() + segment.getY() * segment.getY();
            double b = 2 * (robotToStart.getX() * segment.getX() + robotToStart.getY() * segment.getY());
            double c = robotToStart.getX() * robotToStart.getX() + robotToStart.getY() * robotToStart.getY()
                    - lookaheadDistance * lookaheadDistance;
            double discriminant = b * b - 4 * a * c;
            if(discriminant >= 0) {
                discriminant = Math.sqrt(discriminant);
                double t1 = (-b - discriminant) / (2 * a);
                double t2 = (-b + discriminant) / (2 * a);
                if(t1 >= 0 && t1 <= 1) {
                    lastLookaheadIndex = i;
                    return new Waypoint(path.get(i).getX() + t1 * segment.getX(),
                            path.get(i).getY() + t1 * segment.getY());
                }
                if (t2 >= 0 && t2 <= 1) {
                    lastLookaheadIndex = i;
                    return new Waypoint(path.get(i).getX() + t2 * segment.getX(),
                            path.get(i).getY() + t2 * segment.getY());
                }
            }
        }
        return null;
    }

    private double pathCurvature() {
        Waypoint robot = odometryHandler.getWaypoint();
        Waypoint lookahead = getLookaheadPoint();
        if(lookahead == null) return Double.POSITIVE_INFINITY;
        double yaw = Math.toRadians(90 - odometryHandler.getYaw());
        double slope = Math.tan(yaw);
        double freeTerm = slope * robot.getX() - robot.getY();
        double x = Math.abs(-slope * lookahead.getX() + lookahead.getY() + freeTerm) /
                Math.sqrt(slope * slope + 1); //distance between lookahead point and robot line
        double side = Math.sin(yaw) * (lookahead.getX() - robot.getX()) -
                Math.cos(yaw) * (lookahead.getY() - robot.getY()); //uses cross product to determine side
        if(side == 0) return 0;
        return 2 * x / (lookaheadDistance * lookaheadDistance) * side / Math.abs(side);
    }

    /**
     * Returns the target speeds for left and right as an array.
     * Left speed at index 0, right speed at index 1.
     * The List<Waypoint> has ended when the speeds become {@code Double.POSITIVE_INFINITY}.
     *
     * @return the target side speeds as an array
     */
    public double[] getTargetSpeeds(){
        double velocity = rateLimiter.calculate(closestPoint().getV());
        double pathCurvature = pathCurvature();
        return new double[]{velocity * (2 + pathCurvature * robotWidth) / 2,
                velocity * (2 - pathCurvature * robotWidth) / 2};
    }

    /**
     * Resets the PurePursuitController's local variables so it can be used again.
     * This method should be called right before you start running a path.
     */
    public void reset() {
        lastClosestIndex = 0;
        lastLookaheadIndex = 0;
    }

    /**
     * returns whether the PurePursuitController has finished following the path.
     * @return whether the PurePursuitController has finished following the path
     */
    public boolean done() {
        return closestPoint().equals(path.get(path.size() - 1));
    }
}
