package com.spikes2212.command.drivetrains.commands;

import com.spikes2212.command.drivetrains.TankDrivetrain;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;


public class DriveArcadeWithPID extends CommandBase {

    private final TankDrivetrain drivetrain;
    private PIDSettings pidSettings;
    private PIDController pidController;
    private Supplier<Double> source;
    private double lastTimeNotOnTarget;
    private Supplier<Double> setpoint;
    private Supplier<Double> moveValue;

    public DriveArcadeWithPID(TankDrivetrain drivetrain, PIDSettings pidSettings, Supplier<Double> source,
                              Supplier<Double> setpoint, Supplier<Double> moveValue) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.setpoint = setpoint;
        this.pidSettings = pidSettings;
        this.source = source;
        this.moveValue = moveValue;
        this.pidController = new PIDController(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        this.pidController.setSetpoint(setpoint.get());
    }

    public DriveArcadeWithPID(TankDrivetrain drivetrain, PIDSettings pidSettings, Supplier<Double> source,
                              double setpoint, double moveValue) {
        this(drivetrain, pidSettings, source, () -> setpoint, () -> moveValue);
    }

    /**
     * updates the PIDLoop's setpoint.
     */
    @Override
    public void execute() {
        pidController.setSetpoint(setpoint.get());
        pidController.setTolerance(pidSettings.getTolerance());
        pidController.setPID(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        drivetrain.arcadeDrive(moveValue.get(), pidController.calculate(source.get(), setpoint.get()));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        if(!pidController.atSetpoint()) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }

        return Timer.getFPGATimestamp() - lastTimeNotOnTarget >= pidSettings.getWaitTime();
    }
}
