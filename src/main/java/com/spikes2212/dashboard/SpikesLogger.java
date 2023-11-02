package com.spikes2212.dashboard;

public class SpikesLogger extends RootNamespace {

    private final String key;

    public SpikesLogger() {
        this("SpikesLogger", "Value");
    }

    public SpikesLogger(String name, String key) {
        super(name);
        this.key = key;
    }

    public <T> void log(T output) {
        putString(key, output.toString());
    }
}
