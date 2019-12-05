package com.spikes2212.path;

public class PurePursuitController {
    private OdometryHandler handler;
    private Path path;
    private int lastClosestIndex = 0, lastLookaheadIndex = 0;
    private double lookaheadDistance;

    public PurePursuitController(OdometryHandler handler, Path path, double lookaheadDistance) {
        this.handler = handler;
        this.path = path;
        this.lookaheadDistance = lookaheadDistance;
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

    public Waypoint closestPoint() {
        Waypoint robot = handler.getWaypoint();
        double minDistance = Double.POSITIVE_INFINITY, distance;
        int minIndex;
        for (int i = lastClosestIndex; i < path.getPoints().size(); i++) {
            if ((distance = path.getPoints().get(i).distance(robot)) < minDistance) {
                minIndex = i;
                minDistance = distance;
            }
        }
    }
}
