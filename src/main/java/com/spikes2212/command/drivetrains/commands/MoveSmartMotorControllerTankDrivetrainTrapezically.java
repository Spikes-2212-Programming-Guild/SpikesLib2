package com.spikes2212.command.drivetrains.commands;

import com.spikes2212.command.drivetrains.SmartMotorControllerTankDrivetrain;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.UnifiedControlMode;

import java.util.function.Supplier;

/**
 * A command that moves a {@link SmartMotorControllerTankDrivetrain} by running a trapezoid profile on its
 * masters' control loops.
 *
 * @author Yoel Perman Brilliant
 * @see SmartMotorControllerTankDrivetrain
 * @see MoveSmartMotorControllerTankDrivetrain
 */
public class MoveSmartMotorControllerTankDrivetrainTrapezically extends MoveSmartMotorControllerTankDrivetrain {

    /**
     * The loops' trapezoid profile configurations.
     */
    protected final TrapezoidProfileSettings trapezoidProfileSettings;

    /**
     * Constructs a new instance of {@link MoveSmartMotorControllerTankDrivetrainTrapezically}.
     *
     * @param drivetrain               the {@link SmartMotorControllerTankDrivetrain} this command will run on
     * @param leftPIDSettings          the left side's loop's PID constants
     * @param rightPIDSettings         the right side's loop's PID constants
     * @param feedForwardSettings      the loop's feed forward gains
     * @param leftSetpoint             the setpoint this command should bring the
     *                                 {@link SmartMotorControllerTankDrivetrain}'s left side to
     * @param rightSetpoint            the setpoint this command should bring the
     *                                 {@link SmartMotorControllerTankDrivetrain}'s right side to
     * @param trapezoidProfileSettings the trapezoid profile setting
     */
    public MoveSmartMotorControllerTankDrivetrainTrapezically(SmartMotorControllerTankDrivetrain drivetrain,
                                                              PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                                                              FeedForwardSettings feedForwardSettings,
                                                              Supplier<Double> leftSetpoint,
                                                              Supplier<Double> rightSetpoint,
                                                              TrapezoidProfileSettings trapezoidProfileSettings) {
        super(drivetrain, leftPIDSettings, rightPIDSettings, feedForwardSettings, UnifiedControlMode.TRAPEZOID_PROFILE,
                leftSetpoint, rightSetpoint);
        this.trapezoidProfileSettings = trapezoidProfileSettings;
    }

    @Override
    public void initialize() {
        drivetrain.configureLoop(leftPIDSettings, rightPIDSettings, feedForwardSettings, trapezoidProfileSettings);
    }

    @Override
    public void execute() {
        drivetrain.pidSet(controlMode, leftSetpoint.get(), rightSetpoint.get(), leftPIDSettings, rightPIDSettings,
                feedForwardSettings, trapezoidProfileSettings);
    }
}
