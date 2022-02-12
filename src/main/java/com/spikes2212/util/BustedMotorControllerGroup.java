package com.spikes2212.util;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import java.util.function.Supplier;

/**
 * An extension of the MotorControllerGroup class that has the ability to add a correction to its speed.
 * Best used to correct deviation in the drivetrain.
 *
 * @author Yotam Yizhar
 */
public class BustedMotorControllerGroup extends MotorControllerGroup {

    /**
     * The amount you want to correct the MotorControllerGroup (range 0.1-1).
     */
    private Supplier<Double> correction;

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