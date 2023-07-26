package com.spikes2212.command;

import com.spikes2212.dashboard.Namespace;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * A class that represents a double solenoid.
 *
 * @author Camellia Lami
 * @see DashboardedSubsystem
 */
public class DoubleSolenoidSubsystem extends DashboardedSubsystem {

    /**
     * Creates a {@link DoubleSolenoid}.
     */
    private final DoubleSolenoid doubleSolenoid;

    public DoubleSolenoidSubsystem(Namespace namespace, DoubleSolenoid doubleSolenoid) {
        super(namespace);
        this.doubleSolenoid = doubleSolenoid;
    }

    public DoubleSolenoidSubsystem(String namespaceName, DoubleSolenoid doubleSolenoid) {
        this(new RootNamespace(namespaceName), doubleSolenoid);
    }

    /**
     * Opens the double solenoid.
     *
     * @return An {@link InstantCommand} that opens the solenoid.
     */
    public InstantCommand openSolenoid() {
        return new InstantCommand(() -> doubleSolenoid.set(DoubleSolenoid.Value.kReverse));
    }

    /**
     * Closes the double solenoid.
     *
     * @return An {@link InstantCommand} that closes the solenoid.
     */
    public InstantCommand closeSolenoid() {
        return new InstantCommand(() -> doubleSolenoid.set(DoubleSolenoid.Value.kForward));
    }

    /**
     * Adds any commands or data from this subsystem to the {@link NetworkTable}s.
     */
    @Override
    public void configureDashboard() {
    }
}
