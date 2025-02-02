package com.spikes2212.state;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;

import java.util.HashMap;
import java.util.Map;

@Deprecated(since = "2025", forRemoval = true)
public abstract class LayoutManager<T extends Enum<T>, K extends Enum<K>> {

    private Map<T, ButtonLayout<K>> allLayouts = new HashMap<>();
    private ButtonLayout<K> currentLayout;

    public void addLayout(T mode, ButtonLayout<K> layout) {
        allLayouts.put(mode, layout);
    }

    public Command getCommandFor(K button) {
        return new ProxyCommand(() -> allLayouts.get(currentLayout).getCommandForButton(button));
    }

    public void setLayout(T layout) {
        currentLayout = allLayouts.get(layout);
    }
}
