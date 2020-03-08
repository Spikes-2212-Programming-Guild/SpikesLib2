package com.spikes2212.state;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.HashMap;
import java.util.Map;

public class LayoutManager<T extends Enum<T>> {
    private Map<T, ButtonLayout<Buttons>> allLayouts;
    private ButtonLayout<Buttons> currentLayout;

    public LayoutManager() {
        allLayouts = new HashMap<>();
    }

    public void addLayout(T layout) {
        allLayouts.put(layout, new ButtonLayout<Buttons>());
    }

    public Command getCommandForButton(Buttons button) {
        return allLayouts.get(currentLayout).getCommandForButton(button);
    }

    public void setLayout(T layout) {
        currentLayout = allLayouts.get(layout);
    }

    public void addButtonToLayout(T layout, Buttons button, Command command) {
        allLayouts.get(layout).addButton(button, command);
    }
}

enum Buttons {
    GREEN, BLUE, RED, YELLOW, START, BACK, RT, LT, RB, LB, UP,
    DOWN, LEFT, RIGHT, UPPER_RIGHT, LOWER_RIGHT, UPPER_LEFT, LOWER_LEFT
}
