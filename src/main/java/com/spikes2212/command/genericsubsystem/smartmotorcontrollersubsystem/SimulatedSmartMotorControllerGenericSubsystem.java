package com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem;

import com.spikes2212.util.smartmotorcontrollers.SimulatedSmartMotorControllerWrapper;
import edu.wpi.first.math.system.plant.DCMotor;
import java.util.List;

/**
 * A simulated {@link SmartMotorControllerGenericSubsystem} that steps
 * {@link SimulatedSmartMotorControllerWrapper} instances periodically.
 *
 * @author Adi Yekutiel
 * @see SmartMotorControllerGenericSubsystem
 */
public class SimulatedSmartMotorControllerGenericSubsystem extends SmartMotorControllerGenericSubsystem {

    private static final double DEFAULT_DT_SECONDS = 0.02;

    protected final List<SimulatedSmartMotorControllerWrapper> simulatedMotors;

    public SimulatedSmartMotorControllerGenericSubsystem(String namespaceName,
                                                         SimulatedSmartMotorControllerWrapper... motors) {
        super(namespaceName, motors);
        this.simulatedMotors = List.of(motors);
    }

    public SimulatedSmartMotorControllerGenericSubsystem(String namespaceName, DCMotor motor,
                                                         double gearRatio, double moi) {
        this(namespaceName, new SimulatedSmartMotorControllerWrapper(motor, gearRatio, moi));
    }

    public void update(double dtSeconds) {
        for (SimulatedSmartMotorControllerWrapper motor : simulatedMotors) {
            motor.update(dtSeconds);
        }
    }

    @Override
    public void simulationPeriodic() {
        update(DEFAULT_DT_SECONDS);
    }
}
