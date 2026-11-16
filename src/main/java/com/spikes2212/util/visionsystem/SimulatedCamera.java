package com.spikes2212.util.visionsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import java.util.function.Supplier;

public class SimulatedCamera implements VisionSystem {

    private final Supplier<Pose2d> robotPoseSupplier;

    private final Transform2d cameraToRobot;

    private boolean hasTarget = false;

    private double targetX = 0.0;

    private double targetY = 0.0;

    private double targetArea = 0.0;

    public SimulatedCamera(Supplier<Pose2d> robotPoseSupplier, Transform2d cameraToRobot) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.cameraToRobot = cameraToRobot;
    }

    @Override
    public boolean hasTarget() {
        return hasTarget;
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

    @Override
    public void update(double dtSeconds) {
        Pose2d currentRobotPose = robotPoseSupplier.get();
        Pose2d cameraPose = currentRobotPose.transformBy(cameraToRobot);
    }
}