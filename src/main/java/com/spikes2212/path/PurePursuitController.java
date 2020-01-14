package com.spikes2212.path;

/**
 * This class represents a PurePursuitController.
 * You should the getSpeeds method periodically.
 *
 * @author T
 */
public class PurePursuitController {
    private OdometryHandler handler;
    private Path path;
    private int lastClosestIndex = 0, lastLookaheadIndex = 0;
    private double lookaheadDistance;
    private double robotWidth;
    private Waypoint lookaheadPoint;
    private double distanceTolerance;

    public PurePursuitController(OdometryHandler handler, Path path, double lookaheadDistance,
                                 double distanceTolerance, double robotWidth) {
        this.handler = handler;
        this.path = path;
        this.lookaheadDistance = lookaheadDistance;
        this.distanceTolerance = distanceTolerance;
        this.robotWidth = robotWidth;
        this.lookaheadPoint = path.getPoints().get(0);
    }

    public PurePursuitController(OdometryHandler handler, Path path, double lookaheadDistance,
                                 double robotWidth) {
        this.handler = handler;
        this.path = path;
        this.lookaheadDistance = lookaheadDistance;
        this.distanceTolerance = 0.01;
        this.robotWidth = robotWidth;
        this.lookaheadPoint = path.getPoints().get(0);
    }

    public OdometryHandler getHandler() {
        return handler;
    }

    public void setHandler(OdometryHandler handler) {
        this.handler = handler;
    }

    public Path getPath() {
        return path;
    }

    public void setPath(Path path) {
        this.path = path;
    }

    public double getLookaheadDistance() {
        return lookaheadDistance;
    }

    public void setLookaheadDistance(double lookaheadDistance) {
        this.lookaheadDistance = lookaheadDistance;
    }

    public double getDistanceTolerance() {
        return distanceTolerance;
    }

    public void setDistanceTolerance(double distanceTolerance) {
        this.distanceTolerance = distanceTolerance;
    }

    private Waypoint closestPoint() {
        Waypoint robot = handler.getWaypoint();
        double minDistance = Double.POSITIVE_INFINITY, distance;
        int minIndex = lastClosestIndex;
        for (int i = lastClosestIndex; i < path.getPoints().size(); i++) {
            if ((distance = path.getPoints().get(i).distance(robot)) < minDistance) {
                minIndex = i;
                minDistance = distance;
            }
        }
        lastClosestIndex = minIndex;
        return path.getPoints().get(minIndex);
    }

    private Waypoint getLookaheadPoint(){
        Waypoint robot = handler.getWaypoint();
        for (int i = lastLookaheadIndex; i < path.getPoints().size() - 1; i++) {
            Waypoint segment = new Waypoint(path.getPoints().get(i + 1).getX() - path.getPoints().get(i).getX()
                    , path.getPoints().get(i + 1).getY() - path.getPoints().get(i).getY());
            Waypoint robotToStart = new Waypoint(path.getPoints().get(i).getX() - robot.getX()
                    , path.getPoints().get(i).getY() - robot.getY());
            double a = segment.getX() * segment.getX() + segment.getY() * segment.getY();
            double b = 2 * (robotToStart.getX() * segment.getX() + robotToStart.getY() * segment.getY());
            double c = robotToStart.getX() * robotToStart.getX() + robotToStart.getY() * robotToStart.getY()
                    - lookaheadDistance * lookaheadDistance;
            double discriminant = b * b - 4 * a * c;
            if (discriminant >= 0) {
                discriminant = Math.sqrt(discriminant);
                double t1 = (-b - discriminant) / (2 * a);
                double t2 = (-b + discriminant) / (2 * a);
                if (t1 >= 0 && t1 <= 1) {
                    lastLookaheadIndex = i;
                    lookaheadPoint = new Waypoint(path.getPoints().get(i).getX() + t1 * segment.getX(),
                            path.getPoints().get(i).getY() + t1 * segment.getY());
                    return lookaheadPoint;
                }
                if (t2 >= 0 && t2 <= 1) {
                    lastLookaheadIndex = i;
                    lookaheadPoint = new Waypoint(path.getPoints().get(i).getX() + t2 * segment.getX(),
                            path.getPoints().get(i).getY() + t2 * segment.getY());
                    return lookaheadPoint;
                }
            }
        }
        lookaheadPoint = path.getPoints().get(path.getPoints().size() - 1);
        return lookaheadPoint;
    }

    private double pathCurvature() {
        Waypoint robot = handler.getWaypoint();
        Waypoint lookahead = getLookaheadPoint();
        if(lookahead == null) return Double.POSITIVE_INFINITY;
        double yaw = Math.toRadians(90 - handler.getYaw());
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
     * The Path has ended when the speeds become {@code Double.POSITIVE_INFINITY}.
     * @return the target side speeds as an array
     */
    public double[] getTargetSpeeds(){
        Waypoint closest = closestPoint();
        double pathCurvature = pathCurvature();
        return new double[]{closest.getV() * (2 + pathCurvature * robotWidth) / 2,
                closest.getV() * (2 - pathCurvature * robotWidth) / 2};
    }

    /**
     * Resets the PurePursuitController's local variables so it can be used again.
     * This method should be called right before you start running a path.
     */
    public void reset() {
        lastClosestIndex = 0;
        lastLookaheadIndex = 0;
        lookaheadPoint = path.getPoints().get(0);
    }

    public boolean done() {
        double ratio = handler.getWaypoint().distance(lookaheadPoint) / lookaheadDistance;
        return lookaheadPoint.getV() < 1E-6 && ratio > 1 - distanceTolerance
                && ratio < 1 + distanceTolerance;
    }
}
