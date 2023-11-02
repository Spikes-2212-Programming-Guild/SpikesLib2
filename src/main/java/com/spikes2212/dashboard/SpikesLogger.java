package com.spikes2212.dashboard;

/**
 * a logger class meant to be used with the SpikesLogger desktop app to log values from the robot to a computer in real-time
 */
public class SpikesLogger extends RootNamespace {

    /**
     * the output location
     */
    private final String key;

    /**
     * custom SpikesLogger with custom name and key for the output location
     */
    public SpikesLogger(String name, String key) {
        super(name);
        this.key = key;
    }

    /**
     * default SpikesLogger
     */
    public SpikesLogger() {
        this("SpikesLogger", "Value");
    }

    /**
     * get the output and put it in the SpikesLogger
     */
    public <T> void log(T output) {
        putString(key, output == null ? "null" : output.toString());
    }
}
