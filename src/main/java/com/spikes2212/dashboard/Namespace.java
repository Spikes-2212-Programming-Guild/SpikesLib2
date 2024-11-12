package com.spikes2212.dashboard;

import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.function.Supplier;

/**
 * This is the base interface which all namespaces inherit from.
 */
public interface Namespace {

    /**
     * Adds a Double {@link Supplier} to the namespace, whose value can only be changed by a {@link NetworkTable}s UI,
     * such as the shuffleboard.
     *
     * @param name  the key that will be given to the value
     * @param value the initial value to be added
     * @return a {@link Supplier} with the value set using the {@link NetworkTable}s
     */
    Supplier<Double> addConstantDouble(String name, double value);

    /**
     * Adds an Integer {@link Supplier} to the namespace, whose value can only be changed by a {@link NetworkTable}s UI,
     * such as the shuffleboard.
     *
     * @param name  the key that will be given to the value
     * @param value the initial value to be added
     * @return a {@link Supplier} with the value set using the {@link NetworkTable}s
     */
    Supplier<Integer> addConstantInt(String name, int value);

    /**
     * Adds a String {@link Supplier} to the namespace, whose value can only be changed by a {@link NetworkTable}s UI,
     * such as the shuffleboard.
     *
     * @param name  the key that will be given to the value
     * @param value the initial value to be added
     * @return a {@link Supplier} with the value set using the {@link NetworkTable}s
     */
    Supplier<String> addConstantString(String name, String value);

    /**
     * Adds a {@link ChildNamespace} to this namespace.
     *
     * @param name the name given to the child namespace's name
     * @return the new {@link ChildNamespace} that is created
     */
    ChildNamespace addChild(String name);

    /**
     * Adds a {@link Sendable} to the namespace.
     *
     * @param key   the key that will be given to the value
     * @param value the value to be added
     */
    void putData(String key, Sendable value);

    /**
     * Adds a {@link Command} to the namespace.
     *
     * @param key          the key that will be given to the value
     * @param command      the command to be added
     * @param runOnDisable whether the command should be executable when the robot is disabled
     */
    default void putCommand(String key, Command command, boolean runOnDisable) {
        this.putData(key, command.ignoringDisable(runOnDisable));
    }

    /**
     * Adds a {@link Command} that can run on disable to the namespace.
     *
     * @param key     the key that will be given to the value
     * @param command the command to be added
     */
    default void putCommand(String key, Command command) {
        this.putCommand(key, command, true);
    }

    /**
     * Adds a {@link Runnable} to the namespace that can run as an {@link InstantCommand}.
     *
     * @param key          the key that will be given to the value
     * @param runnable     the runnable value to be added
     * @param runOnDisable whether the instant command should be executable when the robot is disabled
     */
    default void putRunnable(String key, Runnable runnable, boolean runOnDisable) {
        this.putData(key, new InstantCommand(runnable).ignoringDisable(runOnDisable));
    }

    /**
     * Adds a {@link Runnable} that can run on disable as an {@link InstantCommand}.
     *
     * @param key      the key that will be given to the value
     * @param runnable the runnable value to be added
     */
    default void putRunnable(String key, Runnable runnable) {
        putRunnable(key, runnable, true);
    }

    /**
     * Gets a {@link Sendable} from the namespace.
     *
     * @param key the key of the value
     * @return the desired value
     */
    Sendable getSendable(String key);

    /**
     * Adds a String {@link Supplier} to the namespace.
     *
     * @param key   the key that will be given to the value
     * @param value the value to be added
     */
    void putString(String key, Supplier<String> value);

    /**
     * Adds a String value to the namespace.
     *
     * @param key   the key that will be given to the value
     * @param value the value to be added
     */
    default void putString(String key, String value) {
        this.putString(key, () -> value);
    }

    /**
     * Gets a String value from the namespace.
     *
     * @param key the key of the value
     * @return the desired value
     */
    String getString(String key);

    /**
     * Adds a Number {@link Supplier} to the namespace.
     *
     * @param key   the key that will be given to the value
     * @param value the value to be added
     */
    void putNumber(String key, Supplier<? extends Number> value);

    /**
     * Adds a Number value to the namespace.
     *
     * @param key    the key that will be given to the value
     * @param number the value to be added
     */
    default void putNumber(String key, Number number) {
        this.putNumber(key, () -> number);
    }

    /**
     * Gets a Number value from the namespace.
     *
     * @param key the key of the value
     * @return the desired value
     */
    double getNumber(String key);

    /**
     * Adds a Boolean {@link Supplier} to the namespace.
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
     * Adds a set of {@link PIDSettings} values to a designated {@link ChildNamespace}.
     *
     * @param name               the name to be given to the settings and the child namespace
     * @param initialPIDSettings the initial values for the PID settings to be added
     * @return pid settings with the values from the network tables
     */
    default PIDSettings addPIDNamespace(String name, PIDSettings initialPIDSettings) {
        ChildNamespace child = this.addChild(name + " pid");
        Supplier<Double> kP = child.addConstantDouble("kP " + name, initialPIDSettings.getkP());
        Supplier<Double> kI = child.addConstantDouble("kI " + name, initialPIDSettings.getkI());
        Supplier<Double> kD = child.addConstantDouble("kD " + name, initialPIDSettings.getkD());
        Supplier<Double> tolerance = child.addConstantDouble(name + " tolerance", initialPIDSettings.getTolerance());
        Supplier<Double> waitTime = child.addConstantDouble(name + " wait time", initialPIDSettings.getWaitTime());
        Supplier<Double> iZone = child.addConstantDouble(name + " i zone", initialPIDSettings.getIZone());
        return new PIDSettings(kP, kI, kD, tolerance, waitTime, iZone);
    }

    /**
     * Adds a set of {@link FeedForwardSettings} values to a designated {@link ChildNamespace}.
     *
     * @param name                       the name to be given to the settings and the child namespace
     * @param initialFeedForwardSettings the initial feed forward settings to be added
     * @return feed forward settings with the values from the network tables
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
