package com.spikes2212.util;

import com.spikes2212.dashboard.Namespace;
import com.spikes2212.dashboard.RootNamespace;

public interface Dashboarded {

    RootNamespace getRootNamespace();

    /**
     * Updates the {@link Namespace}. Should be called in the {@code robotPeriodic()} method in {@code Robot}.
     */
    default void periodic() {
        getRootNamespace().update();
    }

    void configureDashboard();
}
