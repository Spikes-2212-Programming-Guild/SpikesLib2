package com.spikes2212.util;

/**
 * A simulated component that steps its internal state forward over a time delta.
 *
 * @author Adi Yekutiel
 */
public interface Simulatable {

    double DEFAULT_DT_SECONDS = 0.02;

    void update(double dtSeconds);

    default void update() {
        update(DEFAULT_DT_SECONDS);
    }
}
