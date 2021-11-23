package com.spikes2212.control.noise;

/**
 * an interface which is implemented by classes that handle noise-filtering
 * @author Simon Kharmatsky
 */
public interface NoiseFilter {

    /**
     *a noise-filtering calculation
     */
    double calculate(double measurement);
}
