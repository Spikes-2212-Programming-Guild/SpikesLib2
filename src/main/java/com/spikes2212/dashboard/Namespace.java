package com.spikes2212.dashboard;

import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
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
        this.putData(key, command.ignoringDisable(ignoringDisable));
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

    default void addPIDNamespace(String name, PIDSettings pidSettings) {
        Namespace child = this.addChild(name + " pid");
        Supplier<Double> kP = child.addConstantDouble("kP " + name, pidSettings.getkP());
        Supplier<Double> kI = child.addConstantDouble("kI " + name, pidSettings.getkI());
        Supplier<Double> kD = child.addConstantDouble("kD " + name, pidSettings.getkD());
        Supplier<Double> tolerance = child.addConstantDouble(name + " tolerance", pidSettings.getTolerance());
        Supplier<Double> waitTime = child.addConstantDouble(name + " wait time", pidSettings.getWaitTime());
    }

    default void addFeedForwardNamespace(String name, FeedForwardSettings feedForwardSettings) {
        Namespace child = this.addChild(name + " feed forward");
        Supplier<Double> kS = child.addConstantDouble("kS " + name, feedForwardSettings.getkS());
        Supplier<Double> kV = child.addConstantDouble("kV " + name, feedForwardSettings.getkV());
        Supplier<Double> kA = child.addConstantDouble("kA " + name, feedForwardSettings.getkA());
        Supplier<Double> kG = child.addConstantDouble("kG " + name, feedForwardSettings.getkG());
    }

    void update();
}
