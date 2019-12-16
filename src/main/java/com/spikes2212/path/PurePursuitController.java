package com.spikes2212.path;

/**
 * This class represents a PurePursuitController.
 * You should the getTargetSpeed methods periodically.
 *
 * @author T
 */
public class PurePursuitController {
    private OdometryHandler handler;
    private Path path;
    private int lastClosestIndex = 0, lastLookaheadIndex = 0;
    private double lookaheadDistance;
    private double robotWidth;

    public PurePursuitController(OdometryHandler handler, Path path, double lookaheadDistance,
                                 double robotWidth) {
        this.handler = handler;
        this.path = path;
        this.lookaheadDistance = lookaheadDistance;
        this.robotWidth = robotWidth;
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

    private Waypoint getLookaheadPoint() throws LookaheadPointNotFoundException {
        Waypoint robot = handler.getWaypoint();
        for (int i = lastLookaheadIndex; i < path.getPoints().size(); i++) {
            Waypoint d = new Waypoint(path.getPoints().get(i + 1).getX() - path.getPoints().get(i).getX()
                    , path.getPoints().get(i + 1).getY() - path.getPoints().get(i).getY());
            Waypoint f = new Waypoint(path.getPoints().get(i).getX() - robot.getX()
                    , path.getPoints().get(i).getY() - robot.getY());
            double a = d.getX() * d.getX() + d.getY() * d.getY();
            double b = 2 * (f.getX() * d.getX() + f.getY() * d.getY());
            double c = f.getX() * f.getX() + f.getY() * f.getY() - lookaheadDistance * lookaheadDistance;
            double discriminant = b * b - 4 * a * c;
            if (discriminant >= 0) {
                discriminant = Math.sqrt(discriminant);
                double t1 = (-b - discriminant) / (2 * a);
                double t2 = (-b + discriminant) / (2 * a);
                if (t1 >= 0 && t1 <= 1) {
                    lastLookaheadIndex = i;
                    return new Waypoint(path.getPoints().get(i).getX() + t1 * d.getX(),
                            path.getPoints().get(i).getY() + t1 * d.getY());
                }
                if (t2 >= 0 && t2 <= 1) {
                    lastLookaheadIndex = i;
                    return new Waypoint(path.getPoints().get(i).getX() + t2 * d.getX(),
                            path.getPoints().get(i).getY() + t2 * d.getY());
                }
            }
        }
        throw new LookaheadPointNotFoundException();
    }

    private double pathCurvature() throws LookaheadPointNotFoundException {
        Waypoint robot = handler.getWaypoint();
        Waypoint lookahead = robot;
        try {
            lookahead = getLookaheadPoint();
        } catch (LookaheadPointNotFoundException lpnfe) {
            throw new LookaheadPointNotFoundException(lpnfe);
        }
        double yaw = handler.getYaw();
        double slope = Math.tan(yaw);
        double freeTerm = -slope * robot.getX() - robot.getY();
        double x = Math.abs(-slope * lookahead.getX() + lookahead.getY() + freeTerm) /
                Math.sqrt(slope * slope + 1); //distance between lookahead point and robot line
        double side = Math.sin(yaw) * (lookahead.getX() - robot.getX()) -
                Math.cos(yaw) * (lookahead.getY() - robot.getY()); //uses cross product to determine side
        return x * side / Math.abs(side);
    }

    /**
     * Returns the target speed for the left side of the robot.
     * @return the target speed for the left side of the robot
     * @throws LookaheadPointNotFoundException
     */
    public double getTargetLeftSpeed() throws LookaheadPointNotFoundException {
        return closestPoint().getV() * (2 + pathCurvature() * robotWidth) / 2;
    }

    /**
     * Returns the target speed for the right side of the robot.
     * @return the target speed for the right side of the robot
     * @throws LookaheadPointNotFoundException
     */
    public double getTargetRightSpeed() throws LookaheadPointNotFoundException {
        return closestPoint().getV() * (2 - pathCurvature() * robotWidth) / 2;
    }
}
