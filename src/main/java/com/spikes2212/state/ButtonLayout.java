package com.spikes2212.state;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.HashMap;
import java.util.Map;

@Deprecated(since = "2025", forRemoval = true)
public abstract class ButtonLayout<T extends Enum<T>> {

    private Map<T, Command> buttons = new HashMap<>();

    public void addButton(T button, Command command) {
        buttons.put(button, command);
    }

    public Command getCommandForButton(T button) {
        return buttons.get(button);
    }
}
