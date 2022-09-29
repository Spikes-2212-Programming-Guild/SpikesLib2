package com.spikes2212.command.genericsubsystem;

import com.spikes2212.dashboard.Namespace;
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

    protected MotorControllerGroup motorControllerGroup;

    /**
     * Constructs a new instance of {@link MotoredGenericSubsystem} with the given {@link Namespace}'s name, the given
     * minSpeed supplier, the given maxSpeed supplier and the given {@link MotorController}s.
     *
     * @param namespaceName    the name of the subsystem's namespace
     * @param minSpeed         the minimum speed
     * @param maxSpeed         the maximum speed
     * @param motorControllers the motor controllers in the subsystem
     */
    public MotoredGenericSubsystem(String namespaceName, Supplier<Double> minSpeed, Supplier<Double> maxSpeed,
                                   MotorController... motorControllers) {
        super(namespaceName, minSpeed, maxSpeed);
        this.motorControllerGroup = new MotorControllerGroup(motorControllers);
    }

    /**
     * <p>Constructs a new instance of {@link MotoredGenericSubsystem} with the given
     * minSpeed supplier, the given maxSpeed supplier and the given {@link MotorController}s.</p>
     *
     * <p>This constructor is deprecated.
     * Please use {@link #MotoredGenericSubsystem(String, Supplier, Supplier, MotorController...)}</p>
     *
     * @param minSpeed         the minimum speed
     * @param maxSpeed         the maximum speed
     * @param motorControllers the motor controllers in the subsystem
     */
    @Deprecated(since = "2022", forRemoval = true)
    public MotoredGenericSubsystem(Supplier<Double> minSpeed, Supplier<Double> maxSpeed, String namespaceName,
                                   MotorController... motorControllers) {
        this(namespaceName, minSpeed, maxSpeed, motorControllers);
    }

    /**
     * <p>Constructs a new instance of {@link MotoredGenericSubsystem} with the given {@link Namespace}'s name, the given
     * minSpeed, maxSpeed and the given {@link MotorController}s.</p>
     *
     * @param namespaceName    the name of the subsystem's namespace
     * @param minSpeed         the minimum speed
     * @param maxSpeed         the maximum speed
     * @param motorControllers the motor controllers in the subsystem
     */
    public MotoredGenericSubsystem(String namespaceName, double minSpeed, double maxSpeed,
                                   MotorController... motorControllers) {
        this(namespaceName, () -> minSpeed, () -> maxSpeed, motorControllers);
    }

    /**
     * <p>Constructs a new instance of {@link MotoredGenericSubsystem} with the given
     * minSpeed, the give maxSpeed and the given {@link MotorController}s.</p>
     *
     * <p>This constructor is deprecated.
     * Please use {@link #MotoredGenericSubsystem(String, double, double, MotorController...)} instead.</p>
     *
     * @param minSpeed         the minimum speed
     * @param maxSpeed         the maximum speed
     * @param motorControllers the motor controllers in the subsystem
     */
    @Deprecated(since = "2022", forRemoval = true)
    public MotoredGenericSubsystem(double minSpeed, double maxSpeed, String namespaceName,
                                   MotorController... motorControllers) {
        this(namespaceName, () -> minSpeed, () -> maxSpeed, motorControllers);
    }

    /**
     * <p>Constructs a new instance of {@link MotoredGenericSubsystem} with the given {@link Namespace}'s name and
     * the given {@link MotorController}s.</p>
     *
     * @param namespaceName    the name of the subsystem's namespace
     * @param motorControllers the motor controllers in the subsystem
     */
    public MotoredGenericSubsystem(String namespaceName, MotorController... motorControllers) {
        this(namespaceName, () -> -1.0, () -> 1.0, motorControllers);
    }

    @Override
    protected void apply(double speed) {
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

    /**
     * Adds any commands or data from this subsystem to the dashboard.
     */
    @Override
    public void configureDashboard() {
    }
}
