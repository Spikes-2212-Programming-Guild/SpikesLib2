package com.spikes2212.command;

import com.spikes2212.dashboard.Namespace;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A {@link Subsystem} that controls an applicable double solenoid.
 *
 * @author Camellia Lami
 * @see DashboardedSubsystem
 */
public class DoubleSolenoidSubsystem extends DashboardedSubsystem {

    private final DoubleSolenoid doubleSolenoid;
    private final boolean inverted;

    public DoubleSolenoidSubsystem(Namespace namespace, DoubleSolenoid doubleSolenoid, boolean inverted) {
        super(namespace);
        this.doubleSolenoid = doubleSolenoid;
        this.inverted = inverted;
    }

    public DoubleSolenoidSubsystem(String namespaceName, DoubleSolenoid doubleSolenoid, boolean inverted) {
        this(new RootNamespace(namespaceName), doubleSolenoid, inverted);
    }

    /**
     * Opens the double solenoid.
     *
     * @return an {@link InstantCommand} that opens the solenoid
     */
    public InstantCommand openSolenoid() {
        if (inverted) {
            return new InstantCommand(() -> doubleSolenoid.set(DoubleSolenoid.Value.kForward));
        }
        return new InstantCommand(() -> doubleSolenoid.set(DoubleSolenoid.Value.kReverse));
    }

    /**
     * Closes the double solenoid.
     *
     * @return an {@link InstantCommand} that closes the solenoid
     */
    public InstantCommand closeSolenoid() {
        if (inverted) {
            return new InstantCommand(() -> doubleSolenoid.set(DoubleSolenoid.Value.kReverse));
        }
        return new InstantCommand(() -> doubleSolenoid.set(DoubleSolenoid.Value.kForward));
    }

    /**
     * Adds any commands or data from this subsystem to the {@link NetworkTable}s.
     */
    @Override
    public void configureDashboard() {
    }
}
