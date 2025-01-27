package com.spikes2212.control.noise;

/**
 * An interface which is implemented by classes that handle noise-filtering.
 *
 * @author Simon Kharmatsky
 */
public interface NoiseFilter {

    /**
     * A noise-filtering calculation
     */
    double calculate(double measurement);
}
