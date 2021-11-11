package com.spikes2212.control.noise;

/**
 * @author Simon Kharmatsky
 * an interface which is implemented by classes that handle noise-filtering
 */
public interface NoiseFilter {

    /**
     *a noise-filtering calculation
     */
    double calculate(double measurement);
}
