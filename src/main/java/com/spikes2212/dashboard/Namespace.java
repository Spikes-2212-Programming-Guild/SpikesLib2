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

    /**
     * Adds a double value to the namespace.
     *
     * @param name  the key that will be given to the value
     * @param value the value to be added
     * @return a {@link Supplier} with the most recent value from the network tables.
     */
    Supplier<Double> addConstantDouble(String name, double value);

    /**
     * Adds an integer value to the namespace.
     *
     * @param name  the key that will be given to the value
     * @param value the value to be added
     * @return a {@link Supplier} with the most recent value from the network tables.
     */
    Supplier<Integer> addConstantInt(String name, int value);

    /**
     * Adds a string value to the namespace.
     *
     * @param name  the key that will be given to the value
     * @param value the value to be added
     * @return a {@link Supplier} with the most recent value from the network tables.
     */
    Supplier<String> addConstantString(String name, String value);

    /**
     * Adds a {@link ChildNamespace} to the current namespace.
     *
     * @param name the name given to the child
     * @return the new ChildNamespace that is created
     */
    ChildNamespace addChild(String name);

    /**
     * Adds a sendable value to the namespace.
     *
     * @param key   the key that will be given to the value
     * @param value the value to be added
     */
    void putData(String key, Sendable value);

    /**
     * Adds a command to the namespace.
     *
     * @param key             the key that will be given to the value
     * @param command         the command to be added
     * @param ignoringDisable whether the command should be executable when the robot is disabled
     */
    default void putCommand(String key, Command command, boolean ignoringDisable) {
        this.putData(key, command.ignoringDisable(ignoringDisable));
    }

    /**
     * Adds a command that can run on disable to the namespace.
     *
     * @param key     the key that will be given to the value
     * @param command the command to be added
     */
    default void putCommand(String key, Command command) {
        this.putCommand(key, command, true);
    }

    /**
     * Adds a runnable value to the namespace that can be run as an {@link InstantCommand}.
     *
     * @param key          the key that will be given to the value
     * @param runnable     the runnable value to be added
     * @param runOnDisable whether the command should be executable when the robot is disabled
     */
    default void putRunnable(String key, Runnable runnable, boolean runOnDisable) {
        this.putData(key, new InstantCommand(runnable).ignoringDisable(runOnDisable));
    }

    /**
     * Adds a runnable value to the namespace that can be run as an {@link InstantCommand}.
     *
     * @param key      the key that will be given to the value
     * @param runnable the runnable value to be added
     */
    default void putRunnable(String key, Runnable runnable) {
        putRunnable(key, runnable, true);
    }

    /**
     * Gets a sendable value from the namespace.
     *
     * @param key the key of the value
     * @return the desired value
     */
    Sendable getSendable(String key);

    /**
     * Adds a string to the namespace.
     *
     * @param key   the key that will be given to the value
     * @param value the value to be added
     */
    void putString(String key, Supplier<String> value);

    /**
     * Adds a string to the namespace.
     *
     * @param key   the key that will be given to the value
     * @param value the value to be added
     */
    default void putString(String key, String value) {
        this.putString(key, () -> value);
    }

    /**
     * Gets a string value from the namespace.
     *
     * @param key the key of the value
     * @return the desired value
     */
    String getString(String key);

    /**
     * Adds a number value to the namespace.
     *
     * @param key   the key that will be given to the value
     * @param value the value to be added
     */
    void putNumber(String key, Supplier<? extends Number> value);

    /**
     * Adds a number value to the namespace.
     *
     * @param key    the key that will be given to the value
     * @param number the value to be added
     */
    default void putNumber(String key, Number number) {
        this.putNumber(key, () -> number);
    }

    /**
     * Gets a number value from the namespace.
     *
     * @param key the key of the value
     * @return the desired value
     */
    double getNumber(String key);


    /**
     * Adds a boolean value to the namespace.
     *
     * @param key   the key that will be given to the value
     * @param value the value to be added
     */
    void putBoolean(String key, Supplier<Boolean> value);


    /**
     * Adds a boolean value to the namespace.
     *
     * @param key   the key that will be given to the value
     * @param value the value to be added
     */
    default void putBoolean(String key, boolean value) {
        this.putBoolean(key, () -> value);
    }

    /**
     * Gets a boolean value from the namespace.
     *
     * @param key the key of the value
     * @return the desired value
     */
    boolean getBoolean(String key);

    /**
     * Adds a set of {@link PIDSettings} values to the namespace.
     *
     * @param name               the name to be given to the settings
     * @param initialPIDSettings the initial PID settings to be added
     * @return PID settings with the most recent value from the network tables
     */
    default PIDSettings addPIDNamespace(String name, PIDSettings initialPIDSettings) {
        ChildNamespace child = this.addChild(name + " pid");
        Supplier<Double> kP = child.addConstantDouble("kP " + name, initialPIDSettings.getkP());
        Supplier<Double> kI = child.addConstantDouble("kI " + name, initialPIDSettings.getkI());
        Supplier<Double> kD = child.addConstantDouble("kD " + name, initialPIDSettings.getkD());
        Supplier<Double> tolerance = child.addConstantDouble(name + " tolerance", initialPIDSettings.getTolerance());
        Supplier<Double> waitTime = child.addConstantDouble(name + " wait time", initialPIDSettings.getWaitTime());
        return new PIDSettings(kP, kI, kD, tolerance, waitTime);
    }

    /**
     * Adds a set of {@link FeedForwardSettings} values to the namespace.
     *
     * @param name                       the name to be given to the settings
     * @param initialFeedForwardSettings the initial feed forward settings to be added
     * @return feed forward settings with the most recent value from the network tables
     */
    default FeedForwardSettings addFeedForwardNamespace(String name, FeedForwardSettings initialFeedForwardSettings) {
        ChildNamespace child = this.addChild(name + " feed forward");
        Supplier<Double> kS = child.addConstantDouble("kS " + name, initialFeedForwardSettings.getkS());
        Supplier<Double> kV = child.addConstantDouble("kV " + name, initialFeedForwardSettings.getkV());
        Supplier<Double> kA = child.addConstantDouble("kA " + name, initialFeedForwardSettings.getkA());
        Supplier<Double> kG = child.addConstantDouble("kG " + name, initialFeedForwardSettings.getkG());
        return new FeedForwardSettings(kS, kV, kA, kG);
    }

    /**
     * Updates the namespace.
     */
    void update();
}
