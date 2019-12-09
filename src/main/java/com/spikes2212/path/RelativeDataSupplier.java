package com.spikes2212.path;

import java.util.function.Supplier;

/**
 * This class represents an object with a changing value. </br>
 * The object may be a sensor measuring changing values, time-steps, or any
 * other data supplier.</br>
 * </br>
 *
 * The instances of this class have one method called {@link #get()}, which
 * returns the difference between the current measured value and the last
 * measured value.</br>
 * </br>
 *
 * Use an instance of this class when you are interested in the difference
 * between two measurements, and not in each measurment's singular value.
 *
 * @see Supplier
 *
 * @author T
 */
public class RelativeDataSupplier implements Supplier<Double> {
    /**
     * The {@link Supplier} providing the measured values.
     */
    private Supplier<Double> valueSupplier;
    /**
     * The {@link #valueSupplier}'s value recorded whenever the {@link #get()}
     * method is called,
     */
    private double lastValue;

    /**
     * This constructs a new {@link RelativeDataSupplier} which will return the
     * difference between two values provided by the ValueSupplier parameter
     * using the {@link #get()} method.
     *
     * @param valueSupplier
     *            - The {@link Supplier} providing the measured values.
     *
     * @see Supplier
     */
    public RelativeDataSupplier(Supplier<Double> valueSupplier) {
        this.valueSupplier = valueSupplier;

        initData();
    }
    /**
     *
     * @param valueSupplier the {@link Supplier} to construct a {@link RelativeDataSupplier} from
     * @return the {@link RelativeDataSupplier}
     */
    public static RelativeDataSupplier fromIntegerSupplier(Supplier<Integer> valueSupplier) {
        return new RelativeDataSupplier(() -> (double)(valueSupplier.get()));
    }

    public void initData() {
        lastValue = valueSupplier.get();
    }

    /**
     * This method is the primary utility of the
     * {@link RelativeDataSupplier}.</br>
     * It calculates the difference between the current value of the
     * {@link #valueSupplier} and its recorded value from the last call of the
     * {@link #get()} method.
     */
    public Double get() {
        double currentValue = valueSupplier.get();

        double result = currentValue - lastValue;
        lastValue = currentValue;

        return result;
    }

}