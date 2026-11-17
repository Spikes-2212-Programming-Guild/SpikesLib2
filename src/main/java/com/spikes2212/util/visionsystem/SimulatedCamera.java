package com.spikes2212.util.visionsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * A simulated vision system that computes target offsets and area from field geometry.
 *
 * @author Adi Yekutiel
 * @see VisionSystem
 */
public class SimulatedCamera implements VisionSystem {

    //TODO: those are the numbers I found, but we still need to check them
    private static final double DEFAULT_HORIZONTAL_FOV_DEGREES = 63.3;
    private static final double DEFAULT_VERTICAL_FOV_DEGREES = 49.7;
    private static final double DEFAULT_MAX_RANGE_METERS = 7.0;
    private static final double DEFAULT_TARGET_AREA_METERS_SQUARED = 0.0272;
    private static final double HALF_DIVISOR = 2.0;
    private static final double FRUSTUM_AREA_MULTIPLIER = 4.0;
    private static final double MAX_PERCENTAGE = 100.0;
    private static final double DEFAULT_VALUE = 0.0;

    private final Supplier<Pose2d> robotPoseSupplier;
    private final Transform3d robotToCamera;
    private final List<Pose3d> targets;
    private final double horizontalFovDegrees;
    private final double verticalFovDegrees;
    private final double maxRangeMeters;
    private final double targetAreaMetersSquared;

    private boolean hasTarget;
    private double targetX;
    private double targetY;
    private double targetArea;

    public SimulatedCamera(Supplier<Pose2d> robotPoseSupplier, Transform3d robotToCamera,
                           double horizontalFovDegrees, double verticalFovDegrees,
                           double maxRangeMeters, double targetAreaMetersSquared) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.robotToCamera = robotToCamera;
        this.targets = new ArrayList<>();
        this.horizontalFovDegrees = horizontalFovDegrees;
        this.verticalFovDegrees = verticalFovDegrees;
        this.maxRangeMeters = maxRangeMeters;
        this.targetAreaMetersSquared = targetAreaMetersSquared;
    }

    public SimulatedCamera(Supplier<Pose2d> robotPoseSupplier, Transform3d robotToCamera) {
        this(
                robotPoseSupplier,
                robotToCamera,
                DEFAULT_HORIZONTAL_FOV_DEGREES,
                DEFAULT_VERTICAL_FOV_DEGREES,
                DEFAULT_MAX_RANGE_METERS,
                DEFAULT_TARGET_AREA_METERS_SQUARED
        );
    }

    public SimulatedCamera(Supplier<Pose2d> robotPoseSupplier, Transform2d robotToCamera) {
        this(
                robotPoseSupplier,
                new Transform3d(
                        new Translation3d(robotToCamera.getX(), robotToCamera.getY(), DEFAULT_VALUE),
                        new Rotation3d(DEFAULT_VALUE, DEFAULT_VALUE, robotToCamera.getRotation().getRadians())
                )
        );
    }

    public void addTarget(Pose3d targetPose) {
        targets.add(targetPose);
    }

    public void addTarget(Pose2d targetPose) {
        targets.add(new Pose3d(targetPose));
    }

    public void clearTargets() {
        targets.clear();
    }

    @Override
    public boolean hasTarget() {
        return hasTarget;
    }

    @Override
    public void update(double dtSeconds) {
        Pose3d cameraPose = new Pose3d(robotPoseSupplier.get()).transformBy(robotToCamera);
        double closestDistance = Double.MAX_VALUE;
        boolean found = false;
        double bestTx = DEFAULT_VALUE;
        double bestTy = DEFAULT_VALUE;
        double bestTa = DEFAULT_VALUE;

        for (Pose3d targetPose : targets) {
            Translation3d relativeTranslation = new Transform3d(cameraPose, targetPose).getTranslation();
            double forwardDistance = relativeTranslation.getX();
            if (forwardDistance <= DEFAULT_VALUE) continue;

            double distance = relativeTranslation.getNorm();
            if (distance > maxRangeMeters || distance >= closestDistance) continue;

            double tx = Math.toDegrees(Math.atan2(-relativeTranslation.getY(), forwardDistance));
            double horizontalDistance = Math.hypot(forwardDistance, relativeTranslation.getY());
            double ty = Math.toDegrees(Math.atan2(relativeTranslation.getZ(), horizontalDistance));

            if (Math.abs(tx) <= horizontalFovDegrees / HALF_DIVISOR
                    && Math.abs(ty) <= verticalFovDegrees / HALF_DIVISOR) {
                closestDistance = distance;
                found = true;
                bestTx = tx;
                bestTy = ty;
                bestTa = calculateTargetAreaPercentage(distance);
            }
        }

        this.hasTarget = found;
        this.targetX = bestTx;
        this.targetY = bestTy;
        this.targetArea = bestTa;
    }

    private double calculateTargetAreaPercentage(double distanceMeters) {
        double halfHorizontalFovRad = Math.toRadians(horizontalFovDegrees / HALF_DIVISOR);
        double halfVerticalFovRad = Math.toRadians(verticalFovDegrees / HALF_DIVISOR);
        double frustumArea = FRUSTUM_AREA_MULTIPLIER * distanceMeters * distanceMeters
                * Math.tan(halfHorizontalFovRad) * Math.tan(halfVerticalFovRad);
        if (frustumArea <= DEFAULT_VALUE) return DEFAULT_VALUE;
        return Math.min(MAX_PERCENTAGE, (targetAreaMetersSquared / frustumArea) * MAX_PERCENTAGE);
    }

    @Override
    public double getHorizontalOffsetFromTargetInDegrees() {
        return targetX;
    }

    @Override
    public double getVerticalOffsetFromTargetInDegrees() {
        return targetY;
    }

    @Override
    public double getTargetAreaPercentage() {
        return targetArea;
    }
}
