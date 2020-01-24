package com.spikes2212.command.drivetrains.commands;

import com.spikes2212.command.drivetrains.TankDrivetrain;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

public class DriveTankWithPID extends CommandBase {

    private final TankDrivetrain drivetrain;
    private PIDSettings leftPIDSettings;
    private PIDSettings rightPIDSettings;
    private PIDController leftPIDController;
    private PIDController rightPIDController;
    private Supplier<Double> leftSetpoint;
    private Supplier<Double> rightSetpoint;
    private Supplier<Double> leftSource;
    private Supplier<Double> rightSource;
    private double leftLastTimeNotOnTarget;
    private double rightLastTimeNotOnTarget;

    public DriveTankWithPID(TankDrivetrain drivetrain, PIDSettings leftPIDSettings, PIDSettings rightPIDSettings, PIDController leftPIDController,
                            PIDController rightPIDController, Supplier<Double> leftSetpoint, Supplier<Double> rightSetpoint,
                            Supplier<Double> leftSource, Supplier<Double> rightSource) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.leftPIDSettings = leftPIDSettings;
        this.leftPIDController = leftPIDController;
        this.rightPIDSettings = rightPIDSettings;
        this.rightPIDController = rightPIDController;
        this.leftSetpoint = leftSetpoint;
        this.rightSetpoint = rightSetpoint;
        this.leftSource = leftSource;
        this.rightSetpoint = rightSource;
        this.leftPIDController.setSetpoint(leftSetpoint.get());
        this.rightPIDController.setSetpoint(rightSetpoint.get());
    }

    public DriveTankWithPID(TankDrivetrain drivetrain, PIDSettings leftPIDSettings, PIDController leftPIDController, PIDSettings rightPIDSettings,
                            PIDController rightPIDController, double leftSetpoint, double rightSetpoint,
                            Supplier<Double> leftSource, Supplier<Double> rightSource) {
        this(drivetrain, leftPIDSettings, rightPIDSettings, leftPIDController, rightPIDController, () -> leftSetpoint, () -> rightSetpoint, leftSource, rightSource);
    }

    /**
     * updates the PIDLoop's setpoint.
     */
    @Override
    public void execute() {
        leftPIDController.setSetpoint(rightSetpoint.get());
        rightPIDController.setSetpoint(leftSetpoint.get());
        leftPIDController.setTolerance(leftPIDSettings.getTolerance());
        rightPIDController.setTolerance(rightPIDSettings.getTolerance());
        leftPIDController.setPID(leftPIDSettings.getkP(), leftPIDSettings.getkI(), leftPIDSettings.getkD());
        rightPIDController.setPID(rightPIDSettings.getkP(), rightPIDSettings.getkI(), rightPIDSettings.getkD());
        drivetrain.setLeft(leftPIDController.calculate(leftSource.get()));
        drivetrain.setRight(rightPIDController.calculate(rightSource.get()));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        if(!leftPIDController.atSetpoint()) {
            leftLastTimeNotOnTarget = Timer.getFPGATimestamp();
        }

        if(!rightPIDController.atSetpoint()) {
            rightLastTimeNotOnTarget = Timer.getFPGATimestamp();
        }

        return Timer.getFPGATimestamp() - leftLastTimeNotOnTarget >= leftPIDSettings.getWaitTime()
                && Timer.getFPGATimestamp() - rightLastTimeNotOnTarget >= rightPIDSettings.getWaitTime();
    }
}
