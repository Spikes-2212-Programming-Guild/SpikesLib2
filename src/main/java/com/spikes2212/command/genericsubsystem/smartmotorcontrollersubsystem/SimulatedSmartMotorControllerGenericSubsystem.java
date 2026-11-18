package com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem;

import com.spikes2212.util.Simulatable;
import com.spikes2212.util.smartmotorcontrollers.SimulatedSmartMotorControllerWrapper;
import edu.wpi.first.math.system.plant.DCMotor;
import java.util.List;

/**
 * A simulated {@link SmartMotorControllerGenericSubsystem} that steps
 * {@link SimulatedSmartMotorControllerWrapper} instances periodically.
 *
 * @author Adi Yekutiel
 * @see SmartMotorControllerGenericSubsystem
 * @see Simulatable
 */
public class SimulatedSmartMotorControllerGenericSubsystem extends SmartMotorControllerGenericSubsystem
        implements Simulatable {

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

    @Override
    public void update(double dtSeconds) {
        for (SimulatedSmartMotorControllerWrapper motor : simulatedMotors) {
            motor.update(dtSeconds);
        }
    }

    @Override
    public void simulationPeriodic() {
        update();
    }
}
