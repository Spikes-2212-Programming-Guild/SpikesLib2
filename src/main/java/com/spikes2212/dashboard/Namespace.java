package com.spikes2212.dashboard;

import edu.wpi.first.wpilibj.Sendable;

import java.util.function.Supplier;

public interface Namespace {

    Supplier<Double> addConstantDouble(String name, double value);

    Supplier<Integer> addConstantInt(String name, int value);

    Supplier<String> addConstantString(String name, String value);

    Namespace addChild(String name);

    void putData(String key, Sendable value);

    Sendable getSendable(String key);

    void putString(String key, Supplier<String> value);

    default void putString(String key, String value) {
        this.putString(key, () -> value);
    }

    String getString(String key);

    void putNumber(String key, Supplier<? extends Number> value);

    default void putNumber(String key, Number number) {
        this.putNumber(key, () -> number);
    }


    double getNumber(String key);

    void putBoolean(String key, Supplier<Boolean> value);

    default void putBoolean(String key, boolean value) {
        this.putBoolean(key, () -> value);
    }

    boolean getBoolean(String key);
}
