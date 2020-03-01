package com.spikes2212.control.noise;

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
