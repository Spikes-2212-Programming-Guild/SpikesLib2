package com.spikes2212.control.noise;

public class ExponentialFilter implements NoiseFilter {

    private double w;

    private double previous;


    public ExponentialFilter(double w) {
        this.w = w;
    }


    @Override
    public double calculate(double measurement) {
        double calculation = w * measurement + (1-w) * previous;
        previous = calculation;
        return calculation;
    }
}
