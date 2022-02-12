package com.spikes2212.control.noise;

/**
 * A class that handles noise-filtering by controlling the speed of motors using an average filter.
 *
 * @author Simon Kharmatsky
 */
public class RunningAverageFilter implements NoiseFilter {

    private double sum = 0;
    private int count = 0;

    private void reset() {
        sum = 0;
        count = 0;
    }

    @Override
    public double calculate(double measurement) {
        if (Math.signum(sum) != Math.signum(measurement)) reset();
        sum += measurement;
        count++;
        return sum / count;
    }
}
