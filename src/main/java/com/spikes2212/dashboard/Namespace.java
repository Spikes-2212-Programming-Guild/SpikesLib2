package com.spikes2212.dashboard;

import edu.wpi.first.wpilibj.Sendable;

import java.util.function.Supplier;

public interface Namespace {
    /**
     add double value into the name that was given, in the namespace.
     add int value into the name that was given, in the namespace.
     add string value into the name that was given, in the namespace.
     * @author Tal Sitton
     */
    Supplier<Double> addConstantDouble(String name, double value);

    Supplier<Integer> addConstantInt(String name, int value);

    Supplier<String> addConstantString(String name, String value);

    Namespace addChild(String name);
    /**
     * put data that was given into the key that was given, in the namespace.
     * send the key
     */
    void putData(String key, Sendable value);
    /**
     * getting the sendable value that was inserted under the key that was given.
     */
    Sendable getSendable(String key);

    /**
     * put string that was given into the key that was given, in the namespace.
     */

    void putString(String key, Supplier<String> value);

    default void putString(String key, String value) {
        this.putString(key, () -> value);
    }
    /**
     * getting the string value that was inserted under the key that was given.
     */
    String getString(String key);


    /**
     * put a number from any type that was given into the key that was given, in the namespace.
     */
    void putNumber(String key, Supplier<? extends Number> value);

    default void putNumber(String key, Number number) {
        this.putNumber(key, () -> number);
    }

    /**
     *getting the number that was inserted under the key that was given.
     */
    double getNumber(String key);
    /**
     * put a boolean value into the key that was given, in the namespace.
     */
    void putBoolean(String key, Supplier<Boolean> value);

    default void putBoolean(String key, boolean value) {
        this.putBoolean(key, () -> value);
    }
    /**
     * getting the boolean value that was inserted under the key that was given.
     */
    boolean getBoolean(String key);
}
