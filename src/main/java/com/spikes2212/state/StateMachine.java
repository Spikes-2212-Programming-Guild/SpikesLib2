package com.spikes2212.state;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.HashMap;
import java.util.Map;

public abstract class StateMachine<T extends Enum<T>, State> {
    Map<T, Command> name;

    protected void addTransformation(T state, Command command) {
        name.put(state, command);
    }

    public Command Transform(T state) {
        return name.get(state);
    }

    protected abstract void generateTransformations();

    public StateMachine() {
        generateTransformations();
        name = new HashMap<>();
    }
}
