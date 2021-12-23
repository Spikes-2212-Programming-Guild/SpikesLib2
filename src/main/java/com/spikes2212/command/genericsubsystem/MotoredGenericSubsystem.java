package com.spikes2212.command.genericsubsystem;

import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import java.util.function.Supplier;

/**
 * A motor controlled generic subsystem.
 *
 * @author Ofri Rosenbaum
 * @see GenericSubsystem
 */
public class MotoredGenericSubsystem extends GenericSubsystem {

    private MotorControllerGroup motorControllerGroup;
    protected RootNamespace rootNamespace;

    /**
     * Constructs a new instance of MotoredGenericSubsystem with the given {@code RootNamespace}'s name, the given
     * maxSpeed supplier, the given minSpeed supplier and the {@code MotorController}s.
     *
     * @param minSpeed         the minimum speed
     * @param maxSpeed         the maximum speed
     * @param namespaceName    the subsystem's rootNamespace name
     * @param motorControllers the motor controllers in the subsystem
     */
    public MotoredGenericSubsystem(Supplier<Double> minSpeed, Supplier<Double> maxSpeed, String namespaceName,
                                   MotorController... motorControllers) {
        super(minSpeed, maxSpeed);
        this.rootNamespace = new RootNamespace(namespaceName);
        this.motorControllerGroup = new MotorControllerGroup(motorControllers);
    }

    /**
     * Constructs a new instance of MotoredGenericSubsystem with the given {@code RootNamespace}'s name and
     * the given {@code MotorController}s.
     *
     * @param namespaceName    the subsystem's rootNamespace name
     * @param motorControllers the motor controllers in the subsystem
     */
    public MotoredGenericSubsystem(String namespaceName, MotorController... motorControllers) {
        this(() -> -1.0, () -> 1.0, namespaceName, motorControllers);
    }


    /**
     * Constructs a new instance of MotoredGenericSubsystem with the given {@code RootNamespace}'s name, the given
     * maxSpeed, the given minSpeed and the {@code MotorController}s.
     *
     * @param minSpeed         the minimum speed
     * @param maxSpeed         the maximum speed
     * @param namespaceName    the subsystem's rootNamespace name
     * @param motorControllers the motor controllers in the subsystem
     */
    public MotoredGenericSubsystem(double minSpeed, double maxSpeed, String namespaceName,
                                   MotorController... motorControllers) {
        this(() -> minSpeed, () -> maxSpeed, namespaceName, motorControllers);

    }

    @Override
    public void apply(double speed) {
        motorControllerGroup.set(speed);
    }

    @Override
    public boolean canMove(double speed) {
        return true;
    }

    @Override
    public void stop() {
        motorControllerGroup.stopMotor();
    }

    @Override
    public void periodic() {
        rootNamespace.update();
    }
}
