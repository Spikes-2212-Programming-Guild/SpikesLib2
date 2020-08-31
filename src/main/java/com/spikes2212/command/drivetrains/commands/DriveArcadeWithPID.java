package com.spikes2212.command.drivetrains.commands;

import com.spikes2212.command.drivetrains.TankDrivetrain;
import com.spikes2212.control.PIDFSettings;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

/**
 * A command that moves a drivetrain with a set speed forward and with a PID loop to a certain angle.
 */
public class DriveArcadeWithPID extends CommandBase {
    /**
     * The drivetrain this command operates on.
     */
    private final TankDrivetrain drivetrain;

    /**
     * The PIDF Settings for the turning PIDF loop.
     */
    private PIDFSettings pidfSettings;

    /**
     * The PIDF Controller for the turning PIDF loop.
     */
    private PIDController pidController;

    /**
     * The angle of the drivetrain.
     */
    private Supplier<Double> source;

    /**
     * The last time the drivetrain's angle wasn't within the target range.
     */
    private double lastTimeNotOnTarget;

    /**
     * The angle the drivetrain should reach.
     */
    private Supplier<Double> setpoint;

    /**
     * The speed at which to move the drivetrain forward.
     */
    private Supplier<Double> moveValue;

    public DriveArcadeWithPID(TankDrivetrain drivetrain, Supplier<Double> source, Supplier<Double> setpoint,
                              Supplier<Double> moveValue, PIDFSettings pidfSettings) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.setpoint = setpoint;
        this.pidfSettings = pidfSettings;
        this.source = source;
        this.moveValue = moveValue;
        this.pidController = new PIDController(pidfSettings.getkP(), pidfSettings.getkI(), pidfSettings.getkD());
        this.pidController.setSetpoint(setpoint.get());
    }

    public DriveArcadeWithPID(TankDrivetrain drivetrain, Supplier<Double> source, double setpoint, double moveValue,
                              PIDFSettings pidfSettings) {
        this(drivetrain, source, () -> setpoint, () -> moveValue, pidfSettings);
    }

    /**
     * updates the PIDLoop's setpoint.
     */
    @Override
    public void execute() {
        pidController.setTolerance(pidfSettings.getTolerance());
        pidController.setPID(pidfSettings.getkP(), pidfSettings.getkI(), pidfSettings.getkD());

        drivetrain.arcadeDrive(moveValue.get(), pidController.calculate(source.get(), setpoint.get()) +
                pidfSettings.getkF());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        double currentTime = Timer.getFPGATimestamp();

        if (!pidController.atSetpoint()) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }

        return currentTime - lastTimeNotOnTarget >= pidfSettings.getWaitTime();
    }
}
