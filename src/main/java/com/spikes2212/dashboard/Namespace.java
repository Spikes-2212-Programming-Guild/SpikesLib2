package com.spikes2212.dashboard;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.function.Supplier;

/**
 * This is the base interface which all namespaces inherit from.
 */
public interface Namespace {

    Supplier<Double> addConstantDouble(String name, double value);

    Supplier<Integer> addConstantInt(String name, int value);

    Supplier<String> addConstantString(String name, String value);

    Namespace addChild(String name);

    void putData(String key, Sendable value);

    default void putCommand(String key, Command command, boolean ignoringDisable) {
        this.putData(key, (Sendable)command.ignoringDisable(ignoringDisable));
    }

    default void putCommand(String key, Command command) {
        this.putCommand(key, command, true);
    }

    default void putRunnable(String key, Runnable runnable, boolean runOnDisable) {
        this.putData(key, new InstantCommand(runnable).ignoringDisable(runOnDisable));
    }

    default void putRunnable(String key, Runnable runnable) {
        putRunnable(key, runnable, true);
    }

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

    void update();
}
