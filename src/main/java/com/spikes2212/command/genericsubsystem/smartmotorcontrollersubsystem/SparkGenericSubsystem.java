package com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem;

import com.revrobotics.spark.SparkBase;
import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A {@link Subsystem} which consists of a master {@link SparkBase} motor controller that runs control
 * loops and additional {@link SparkBase} motor controllers that follow it.
 *
 * @author Yoel Perman Brilliant
 * @see DashboardedSubsystem
 * @see SmartMotorControllerGenericSubsystem
 */
@Deprecated(since = "2025", forRemoval = true)
public class SparkGenericSubsystem extends SmartMotorControllerGenericSubsystem {

    /**
     * Constructs a new instance of {@link SparkGenericSubsystem}.
     *
     * @param namespaceName the name of the subsystem's namespace
     */
    public SparkGenericSubsystem(String namespaceName, SparkWrapper... sparkWrappers) {
        super(namespaceName, sparkWrappers);
    }
}
