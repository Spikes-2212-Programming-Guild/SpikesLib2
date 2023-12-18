package com.spikes2212.dashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * A logger class meant to be used with the SpikesLogger desktop app to log values from the robot to a computer in real-time.
 * Uses a {@link NetworkTable} to communicate with the computer.
 *
 * @author TzintzeneT
 */
public class SpikesLogger extends RootNamespace {

    /**
     * NetworkTables key to use for the output.
     */
    private final String key;

    /**
     * Creates a SpikesLogger instance with custom name and key for the output location.
     */
    public SpikesLogger(String name, String key) {
        super(name);
        this.key = key;
    }

    /**
     * Creates a default SpikesLogger instance with custom key (name = "SpikesLogger").
     */
    public SpikesLogger(String key) {
        this("SpikesLogger", key);
    }

    /**
     * Creates a default SpikesLogger instance (name = "SpikesLogger", key = "output").
     */
    public SpikesLogger() {
        this("SpikesLogger", "output");
    }

    /**
     * Logs the provided output to the NetworkTables and the SpikesLogger app.
     *
     * @param output the data to be logged
     */
    public <T> void log(T output) {
        putString(key, output == null ? "null" : output.toString());
    }


    /**
     * Returns a command that logs the provided output to the NetworkTables and the SpikesLogger app.
     *
     * @param output the data to be logged
     * @return a command that logs the output
     */
    public <T> Command logCommand(T output) {
        return new InstantCommand(() -> log(output));
    }
}
