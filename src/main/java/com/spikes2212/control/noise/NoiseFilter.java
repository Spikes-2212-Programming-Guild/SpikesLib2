package com.spikes2212.control.noise;

/**
 * An interface which is implemented by classes that handle noise-filtering.
 *
 * @author Simon Kharmatsky
 */
@Deprecated(since = "2025", forRemoval = true)
public interface NoiseFilter {

    /**
     * A noise-filtering calculation
     */
    double calculate(double measurement);
}
