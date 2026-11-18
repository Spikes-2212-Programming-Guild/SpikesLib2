package com.spikes2212.util.visionsystem;

import com.spikes2212.util.Simulatable;

public interface VisionSystem extends Simulatable {

    boolean hasTarget();

    double getHorizontalOffsetFromTargetInDegrees();

    double getVerticalOffsetFromTargetInDegrees();

    double getTargetAreaPercentage();

    void update(double dtSeconds);
}
