package com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.List;

/**
 * A {@link Subsystem} which consists of a master {@link TalonFX} motor controller that runs control
 * loops and additional {@link TalonFX} motor controllers that follow it.
 *
 * @author Netta Hamalish
 * @see DashboardedSubsystem
 * @see SmartMotorControllerGenericSubsystem
 */
public class TalonFXGenericSubsystem extends DashboardedSubsystem implements SmartMotorControllerGenericSubsystem {

    /**
     * The TalonFX which runs the loops.
     */
    protected final TalonFX master;

    /**
     * Additional TalonFXs that follow the master.
     */
    protected final List<TalonFX> slaves;

    /**
     * Constructs a new instance of {@link TalonFXGenericSubsystem}.
     *
     * @param namespaceName the name of the subsystem's namespace
     * @param master        the TalonFX which runs the loops
     * @param slaves        additional TalonFXs that follow the master
     */
    public TalonFXGenericSubsystem(String namespaceName, TalonFX master, TalonFX... slaves) {
        super(namespaceName);
        this.master = master;
        this.slaves = List.of(slaves);
    }

    /**
     * Adds any data or commands to the {@link NetworkTable}s, which can be accessed using the {@link Shuffleboard}.
     */
    @Override
    public void configureDashboard() {
    }

    @Override
    public void configPIDF(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings) {
        Slot0Configs config = new Slot0Configs();
        config.kP = pidSettings.getkP();
        config.kI = pidSettings.getkI();
        config.kD = pidSettings.getkD();
        config.kS = feedForwardSettings.getkS();
        config.kV = feedForwardSettings.getkV();
        config.kA = feedForwardSettings.getkA();
        config.kG = feedForwardSettings.getkG();
        master.getConfigurator().apply(config);
    }

    @Override
    public void configureTrapezoid(TrapezoidProfileSettings settings) {
        MotionMagicConfigs config = new MotionMagicConfigs();
        config.MotionMagicAcceleration = settings.getAccelerationRate();
        config.MotionMagicCruiseVelocity = settings.getMaxVelocity();
        config.MotionMagicJerk = settings.getCurve();
        master.getConfigurator().apply(config);
    }

    @Override
    public void configureLoop(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings,
                              TrapezoidProfileSettings trapezoidProfileSettings) {
        master.getConfigurator().apply(new TalonFXConfiguration());
        configPIDF(pidSettings, feedForwardSettings);
        configureTrapezoid(trapezoidProfileSettings);
    }

    @Override
    public void pidSet(UnifiedControlMode controlMode, double setpoint, PIDSettings pidSettings,
                       FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings) {
        configPIDF(pidSettings, feedForwardSettings);
        configureTrapezoid(trapezoidProfileSettings);
        ControlRequest request = switch (controlMode) {
            case CURRENT -> new TorqueCurrentFOC(setpoint);
            case PERCENT_OUTPUT -> new DutyCycleOut(setpoint);
            case TRAPEZOID_PROFILE -> new MotionMagicDutyCycle(setpoint);
            case MOTION_PROFILING -> throw new UnsupportedOperationException("Motion Profiling is not yet implemented in SpikesLib2!");
            case VOLTAGE -> new VoltageOut(setpoint);
            case VELOCITY -> new VelocityDutyCycle(setpoint);
            case POSITION -> new PositionDutyCycle(setpoint);
        };
        master.setControl(request);
        slaves.forEach(s -> s.setControl(new Follower(master.getDeviceID(), false)));
    }

    /**
     * Stops any control loops running on the TalonFX.
     */
    @Override
    public void finish() {
        master.stopMotor();
    }

    /**
     * Checks whether the loop is currently on the target setpoint. <br>
     * This method, as is, <b>does not</b> cover every case and should be overridden if necessary.
     *
     * @param controlMode the loop's control type (e.g. voltage, velocity, position...)
     * @param tolerance   the maximum difference from the target to still be considered on target
     * @param setpoint    the wanted setpoint
     * @return {@code true} when on target setpoint, {@code false} otherwise
     */
    @Override
    public boolean onTarget(UnifiedControlMode controlMode, double tolerance, double setpoint) {
        double value = switch (controlMode) {
            case VELOCITY -> master.getVelocity().getValue().in(Units.RotationsPerSecond);
            case POSITION, MOTION_PROFILING, TRAPEZOID_PROFILE -> master.getPosition().getValue().in(Units.Revolutions);
            case CURRENT -> master.getTorqueCurrent().getValue().in(Units.Amps);
            case PERCENT_OUTPUT -> master.get();
            case VOLTAGE -> master.getMotorVoltage().getValue().in(Units.Volts);
        };
        return Math.abs(value - setpoint) <= tolerance;
    }
}
