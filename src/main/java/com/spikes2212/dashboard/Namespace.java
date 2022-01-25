package com.spikes2212.dashboard;

import edu.wpi.first.wpilibj.Sendable;

import java.util.function.Supplier;

public interface Namespace {
    /**
     A supplier that adds double value into the name that was given, in the namespace.
     A supplier that adds int value into the name that was given, in the namespace.
     A supplier that adds string value into the name that was given, in the namespace.
     * @author Tal Sitton
     */
    Supplier<Double> addConstantDouble(String name, double value);

    Supplier<Integer> addConstantInt(String name, int value);

    Supplier<String> addConstantString(String name, String value);

    Namespace addChild(String name);
    /**
     * A command that puts data that was given into the key that was given, in the namespace.
     * send the key
     */
    void putData(String key, Sendable value);
    /**
     * A command that gets the sendable value that was inserted under the key that was given.
     */
    Sendable getSendable(String key);

    /**
     * A command that puts string that was given into the key that was given, in the namespace.
     */

    void putString(String key, Supplier<String> value);

    default void putString(String key, String value) {
        this.putString(key, () -> value);
    }
    /**
     * A command that gets the string value that was inserted under the key that was given.
     */
    String getString(String key);


    /**
     * A command that puts a number from any type that was given into the key that was given, in the namespace.
     */
    void putNumber(String key, Supplier<? extends Number> value);

    default void putNumber(String key, Number number) {
        this.putNumber(key, () -> number);
    }

    /**
     *A command that gets the number that was inserted under the key that was given.
     */
    double getNumber(String key);
    /**
     * A command that puts a boolean value into the key that was given, in the namespace.
     */
    void putBoolean(String key, Supplier<Boolean> value);

    default void putBoolean(String key, boolean value) {
        this.putBoolean(key, () -> value);
    }
    /**
     * A command that gets the boolean value that was inserted under the key that was given.
     */
    boolean getBoolean(String key);
}
