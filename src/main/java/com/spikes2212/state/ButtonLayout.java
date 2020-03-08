package com.spikes2212.state;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.HashMap;
import java.util.Map;

public class ButtonLayout<T extends Enum<T>> {
    private Map<T, Command> layout;

    public ButtonLayout() {
        layout = new HashMap<>();
    }

    public void addButton(T button, Command command) {
        layout.put(button, command);
    }

    public Command getCommandForButton(T button) {
        return layout.get(button);
    }
}
