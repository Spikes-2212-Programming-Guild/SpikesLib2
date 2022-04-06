package com.spikes2212.command;

import com.spikes2212.dashboard.Namespace;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A {@link Subsystem} that includes a {@link Namespace}.
 */
public abstract class DashboardedSubsystem extends SubsystemBase {

    protected RootNamespace rootNamespace;

    public DashboardedSubsystem(String namespaceName) {
        this.rootNamespace = new RootNamespace(namespaceName);
    }

    /**
     * Updates the {@link Namespace}. Should be called in the {@code robotPeriodic()} method in {@code Robot}.
     */
    @Override
    public void periodic() {
        rootNamespace.update();
    }

    public abstract void configureDashboard();
}
