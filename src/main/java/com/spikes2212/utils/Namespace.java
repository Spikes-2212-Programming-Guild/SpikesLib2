package com.spikes2212.utils;

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

    String getString(String key);

    void putNumber(String key, Supplier<? extends Number> value);

    double getNumber(String key);

    void putBoolean(String key, Supplier<Boolean> value);

    boolean getBoolean(String key);
}
