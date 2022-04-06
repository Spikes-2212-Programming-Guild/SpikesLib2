package com.spikes2212.command;

import com.spikes2212.dashboard.Namespace;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.Dashboarded;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A {@link Subsystem} that includes a {@link Namespace}.
 */
public abstract class DashboardedSubsystem extends SubsystemBase implements Dashboarded {

    protected RootNamespace rootNamespace;

    public DashboardedSubsystem(String namespaceName) {
        this.rootNamespace = new RootNamespace(namespaceName);
    }

    /**
     * Add any commands or data from this subsystem to the dashboard.
     */
    public abstract void configureDashboard();

    @Override
    public RootNamespace getRootNamespace() {
        return rootNamespace;
    }

    @Override
    public void periodic() {
        Dashboarded.super.periodic();
    }
}
