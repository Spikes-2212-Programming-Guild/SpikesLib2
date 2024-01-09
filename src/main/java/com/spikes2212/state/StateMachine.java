package com.spikes2212.state;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;

import java.util.HashMap;
import java.util.Map;

public abstract class StateMachine<T extends Enum<T>> {

    private Map<T, Command> transformations;
    private T state;

    public StateMachine(T initialState) {
        setState(initialState);
        transformations = new HashMap<>();
        generateTransformations();
    }

    protected abstract void generateTransformations();

    protected void addTransformation(T state, Command command) {
        transformations.put(state, new InstantCommand(() -> setState(state)).andThen(command));
    }

    public Command getTransformationFor(T state) {
        return new ProxyCommand(transformations.get(state));
    }

    protected void setState(T state) {
        this.state = state;
    }

    public T getState() {
        return state;
    }
}
