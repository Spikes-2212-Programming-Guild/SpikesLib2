package com.spikes2212.util;

import com.spikes2212.dashboard.RootNamespace;

public interface Dashboarded {

    RootNamespace getRootNamespace();

    default void periodic() {
        getRootNamespace().update();
    }

    void configureDashboard();
}
