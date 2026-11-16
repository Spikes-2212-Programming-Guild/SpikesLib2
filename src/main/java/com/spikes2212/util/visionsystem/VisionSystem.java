package com.spikes2212.util.visionsystem;

public interface VisionSystem {

    boolean hasTarget();

    double getHorizontalOffsetFromTargetInDegrees();

    double getVerticalOffsetFromTargetInDegrees();

    double getTargetAreaPercentage();

    void update(double dtSeconds);
}
