package com.spikes2212.command;

import com.spikes2212.dashboard.Namespace;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * A {@link Subsystem} that includes a {@link Namespace}, which lets you see and configure data using the
 * {@link SmartDashboard}.
 *
 * @author Yoel Perman Brilliant
 */
public abstract class DashboardedSubsystem extends SubsystemBase {

    protected final Namespace namespace;

    public DashboardedSubsystem(Namespace namespace) {
        this.namespace = namespace;
    }

    public DashboardedSubsystem(String namespaceName) {
        this(new RootNamespace(namespaceName));
    }

    /**
     * Updates the {@link Namespace}. Should be called in the {@code robotPeriodic()} method in {@code Robot}.
     */
    @Override
    public void periodic() {
        namespace.update();
    }

    public abstract void configureDashboard();

    /**
     * Should be used inside a constructor to get the name of the class of the object that is being created.
     * The method achieves that by finding the earliest constructor in the method call stack.
     * @param defaultName default return value in case the method, for any reason, does not find a constructor.
     * @return name of the class
     */
    protected static String getClassName(String defaultName) {
        List<StackTraceElement> elements = Arrays.asList(Thread.currentThread().getStackTrace());
        Collections.reverse(elements);
        for (StackTraceElement element : elements) {
            if (element.toString().contains(".<init>"))
                return element.getClassName();
        }
        return defaultName;
    }
}
