package com.spikes2212.control.noise;

import java.util.function.Supplier;

public class NoiseReducer implements Supplier<Double> {

    private Supplier<Double> source;

    private NoiseFilter filter;

    public NoiseReducer(Supplier<Double> source, NoiseFilter filter) {
        this.source = source;
        this.filter = filter;
    }

    @Override
    public Double get() {
        return filter.calculate(source.get());
    }
}
