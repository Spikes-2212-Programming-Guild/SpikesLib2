package com.spikes2212.util;

import java.util.ArrayList;
import java.util.List;

/**
 * A manager that coordinates and updates registered {@link Simulatable} components.
 *
 * @author Adi Yekutiel
 * @see Simulatable
 */
public class SimulationManager implements Simulatable {

    private final List<Simulatable> simulatables;

    public SimulationManager(List<Simulatable> simulatables) {
        this.simulatables = new ArrayList<>(simulatables);
    }

    public SimulationManager(Simulatable... simulatables) {
        this(List.of(simulatables));
    }

    public void register(Simulatable... simulatables) {
        this.simulatables.addAll(List.of(simulatables));
    }

    public void unregister(Simulatable simulatable) {
        simulatables.remove(simulatable);
    }

    public void clear() {
        simulatables.clear();
    }

    @Override
    public void update(double dtSeconds) {
        for (Simulatable simulatable : simulatables) {
            simulatable.update(dtSeconds);
        }
    }
}
