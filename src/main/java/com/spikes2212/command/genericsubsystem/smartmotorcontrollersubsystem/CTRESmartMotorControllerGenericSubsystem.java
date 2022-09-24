package com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IFollower;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.spikes2212.command.genericsubsystem.GenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import java.util.List;


public class CTRESmartMotorControllerGenericSubsystem<T extends BaseMotorController & MotorController>
        extends GenericSubsystem implements SmartMotorControllerSubsystem {

    private static final int SLOT = 0;
    private static final int TIME_OUT_MS = 1;

    protected final T master;
    protected final List<? extends IFollower> slaves;

    public CTRESmartMotorControllerGenericSubsystem(String namespaceName, double minSpeed, double maxSpeed, T master,
                                                    IFollower... slaves) {
        super(namespaceName, minSpeed, maxSpeed);
        this.master = master;
        this.slaves = List.of(slaves);
        this.slaves.forEach(s -> s.follow(master));
    }

    @Override
    public void configureDashboard() {

    }

    @Override
    protected void apply(double speed) {
        master.set(speed);
    }

    @Override
    public boolean canMove(double speed) {
        return true;
    }

    @Override
    public void stop() {
        master.stopMotor();
    }

    @Override
    public void configPIDF(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings) {
        master.config_kP(SLOT, pidSettings.getkP());
        master.config_kI(SLOT, pidSettings.getkI());
        master.config_kD(SLOT, pidSettings.getkD());
        master.config_kF(SLOT, feedForwardSettings.getkV());
    }

    @Override
    public void configureTrapezoid(TrapezoidProfileSettings settings) {
        master.configMotionAcceleration(settings.getAccelerationRate());
        master.configMotionCruiseVelocity(settings.getMaxVelocity());
        master.configMotionSCurveStrength(settings.getCurve());
    }

    @Override
    public void configureLoop(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings,
                              TrapezoidProfileSettings trapezoidProfileSettings) {
        master.configFactoryDefault();
        configPIDF(pidSettings, feedForwardSettings);
        configureTrapezoid(trapezoidProfileSettings);
    }

    @Override
    public void configureLoop(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings) {
       configureLoop(pidSettings, feedForwardSettings, TrapezoidProfileSettings.EMPTY_TRAPEZOID_PROFILE_SETTINGS);
    }

    @Override
    public void pidSet(ControlMode controlMode, double setpoint, PIDSettings pidSettings,
                       FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings) {
        configPIDF(pidSettings, feedForwardSettings);
        configureTrapezoid(trapezoidProfileSettings);
        master.set(controlMode, setpoint);
    }

    @Override
    public void finish() {
        stop();
    }
}
