package com.spikes2212.command;

import com.spikes2212.dashboard.Namespace;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DoubleSolenoidSubsystem extends DashboardedSubsystem {

    private final DoubleSolenoid doubleSolenoid;

    public DoubleSolenoidSubsystem(Namespace namespace, DoubleSolenoid doubleSolenoid) {
        super(namespace);
        this.doubleSolenoid = doubleSolenoid;
    }

    public DoubleSolenoidSubsystem(String namespaceName, DoubleSolenoid doubleSolenoid) {
        this(new RootNamespace(namespaceName), doubleSolenoid);
    }

    public InstantCommand openSolenoid() {
        return new InstantCommand(() -> doubleSolenoid.set(DoubleSolenoid.Value.kReverse));
    }

    public InstantCommand closeSolenoid() {
        return new InstantCommand(() -> doubleSolenoid.set(DoubleSolenoid.Value.kForward));
    }

    @Override
    public void configureDashboard() {

    }
}
