package com.spikes2212.util;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import java.util.function.Supplier;

/**
 * An extension of the {@code MotorControllerGroup} class that has the ability to add a correction to its speed.<br>
 * Best used to correct deviation in the drivetrain.
 *
 * @author Yotam Yizhar
 */
public class BustedMotorControllerGroup extends MotorControllerGroup {

    /**
     * The wanted amount to correct the {@code MotorControllerGroup} (range 0.1-1).
     */
    protected final Supplier<Double> correction;

    public BustedMotorControllerGroup(Supplier<Double> correction, MotorController motorController, MotorController... motorControllers) {
        super(motorController, motorControllers);
        this.correction = correction;
    }

    public BustedMotorControllerGroup(double correction, MotorController motorController, MotorController... motorControllers) {
        this(() -> correction, motorController, motorControllers);
    }

    @Override
    public void set(double speed) {
        super.set(speed * correction.get());
    }
}
