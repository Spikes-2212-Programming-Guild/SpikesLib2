package com.spikes2212.command.drivetrains.tankdrivetrains.smartmotorcontrollerdrivetrain;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.spikes2212.command.drivetrains.tankdrivetrains.TankDrivetrain;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import java.util.List;

/**
 * A {@link TankDrivetrain} which consists of a master {@link TalonFX} controller that can run control loops and additional
 * {@link TalonFX} motor controllers that follow it.
 *
 * @author Camilia Lami
 * @see TankDrivetrain
 * @see SmartMotorControllerTankDrivetrain
 */
public class TalonFXTankDrivetrain extends TankDrivetrain implements SmartMotorControllerTankDrivetrain {

    /**
     * The motor controller that runs the left side's loops.
     */
    protected final TalonFX leftMaster;

    /**
     * The motor controller that runs the right side's loops.
     */
    protected final TalonFX rightMaster;

    /**
     * Additional motor controllers that follow the left master.
     */
    protected final List<? extends TalonFX> leftSlaves;

    /**
     * Additional motor controllers that follow the right master.
     */
    protected final List<? extends TalonFX> rightSlaves;

    /**
     * Constructs a new instance of {@link TalonFXTankDrivetrain}.
     *
     * @param namespaceName the name of the drivetrain's namespace
     * @param leftMaster    the motor controller that runs the left side's loops
     * @param leftSlaves    additional motor controllers that follow the left master
     * @param rightMaster   the motor controller that runs the right side's loops
     * @param rightSlaves   additional motor controllers that follow the right master
     */
    public TalonFXTankDrivetrain(String namespaceName, TalonFX leftMaster,
                                 List<? extends TalonFX> leftSlaves, TalonFX rightMaster,
                                 List<? extends TalonFX> rightSlaves) {
        super(namespaceName, (MotorController) leftMaster, (MotorController) rightMaster);
        this.leftMaster = leftMaster;
        this.leftSlaves = leftSlaves;
        this.rightMaster = rightMaster;
        this.rightSlaves = rightSlaves;
        rightController.setInverted(false);
        MotorOutputConfigs configuration = new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);
        rightMaster.getConfigurator().apply(configuration);
        rightSlaves.forEach(s -> s.getConfigurator().apply(configuration));
    }

    /**
     * Constructs a new instance of {@link TalonFXTankDrivetrain}, where each side has two
     * motor controllers.
     *
     * @param namespaceName the name of the drivetrain's namespace
     * @param leftMaster    the motor controller that runs the left side's loops
     * @param leftSlave     an additional motor controller that follows the left master
     * @param rightMaster   the motor controller that runs the right side's loops
     * @param rightSlave    an additional motor controller that follows the right master
     */
    public TalonFXTankDrivetrain(String namespaceName, TalonFX leftMaster, TalonFX leftSlave, TalonFX rightMaster,
                                 TalonFX rightSlave) {
        this(namespaceName, leftMaster, List.of(leftSlave), rightMaster, List.of(rightSlave));
    }

    /**
     * Adds any data or commands to the {@link NetworkTable}s, which can be accessed using the {@link Shuffleboard}.
     */
    @Override
    public void configureDashboard() {
    }

    /**
     * Configures the loops' PID constants and feed forward gains.
     */
    @Override
    public void configPIDF(PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                           FeedForwardSettings feedForwardSettings) {
        Slot0Configs leftConfig = new Slot0Configs();
        Slot0Configs rightConfig = new Slot0Configs();
        leftConfig.kP = leftPIDSettings.getkP();
        leftConfig.kI = leftPIDSettings.getkI();
        leftConfig.kD = leftPIDSettings.getkD();
        leftConfig.kS = feedForwardSettings.getkS();
        leftConfig.kV = feedForwardSettings.getkV();
        leftConfig.kA = feedForwardSettings.getkA();
        leftConfig.kG = feedForwardSettings.getkG();
        leftMaster.getConfigurator().apply(leftConfig);
        rightConfig.kP = rightPIDSettings.getkP();
        rightConfig.kI = rightPIDSettings.getkI();
        rightConfig.kD = rightPIDSettings.getkD();
        rightConfig.kS = feedForwardSettings.getkS();
        rightConfig.kV = feedForwardSettings.getkV();
        rightConfig.kA = feedForwardSettings.getkA();
        rightConfig.kG = feedForwardSettings.getkG();
        rightMaster.getConfigurator().apply(rightConfig);
    }

    /**
     * Configures the loops' trapezoid profile settings.
     */
    @Override
    public void configureTrapezoid(TrapezoidProfileSettings settings) {
        MotionMagicConfigs config = new MotionMagicConfigs();
        config.MotionMagicAcceleration = settings.getAccelerationRate();
        config.MotionMagicCruiseVelocity = settings.getMaxVelocity();
        config.MotionMagicJerk = settings.getCurve();
        leftMaster.getConfigurator().apply(config);
        rightMaster.getConfigurator().apply(config);
    }

    /**
     * Configures the loops' settings.
     */
    @Override
    public void configureLoop(PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                              FeedForwardSettings feedForwardSettings,
                              TrapezoidProfileSettings trapezoidProfileSettings) {
        leftMaster.getConfigurator().apply(new TalonFXConfiguration());
        rightMaster.getConfigurator().apply(new TalonFXConfiguration());
        configPIDF(leftPIDSettings, rightPIDSettings, feedForwardSettings);
        configureTrapezoid(trapezoidProfileSettings);
    }

    /**
     * Updates any control loops running on each side's motor controllers.
     *
     * @param controlMode              the loops' control mode (e.g. voltage, velocity, position...)
     * @param leftSetpoint             the left side's loop's target setpoint
     * @param rightSetpoint            the right side's loop's target setpoint
     * @param leftPIDSettings          the left side's PID constants
     * @param rightPIDSettings         the right side's PID constants
     * @param feedForwardSettings      the feed forward gains
     * @param trapezoidProfileSettings the trapezoid profile settings
     */
    @Override
    public void pidSet(UnifiedControlMode controlMode, double leftSetpoint, double rightSetpoint,
                       PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                       FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings) {
        configPIDF(leftPIDSettings, rightPIDSettings, feedForwardSettings);
        ControlRequest leftRequest = switch (controlMode) {
            case CURRENT -> new TorqueCurrentFOC(leftSetpoint);
            case PERCENT_OUTPUT -> new DutyCycleOut(leftSetpoint);
            case TRAPEZOID_PROFILE -> new MotionMagicDutyCycle(leftSetpoint);
            case MOTION_PROFILING -> throw new UnsupportedOperationException("Motion Profiling is not yet implemented in SpikesLib2!");
            case VOLTAGE -> new VoltageOut(leftSetpoint);
            case VELOCITY -> new VelocityDutyCycle(leftSetpoint);
            case POSITION -> new PositionDutyCycle(leftSetpoint);
        };
        ControlRequest rightRequest = switch (controlMode) {
            case CURRENT -> new TorqueCurrentFOC(rightSetpoint);
            case PERCENT_OUTPUT -> new DutyCycleOut(rightSetpoint);
            case TRAPEZOID_PROFILE -> new MotionMagicDutyCycle(rightSetpoint);
            case MOTION_PROFILING -> throw new UnsupportedOperationException("Motion Profiling is not yet implemented in SpikesLib2!");
            case VOLTAGE -> new VoltageOut(rightSetpoint);
            case VELOCITY -> new VelocityDutyCycle(rightSetpoint);
            case POSITION -> new PositionDutyCycle(rightSetpoint);
        };
        leftMaster.setControl(leftRequest);
        rightMaster.setControl(rightRequest);
        leftSlaves.forEach(s -> s.setControl(new Follower(leftMaster.getDeviceID(), false)));
        rightSlaves.forEach(s -> s.setControl(new Follower(rightMaster.getDeviceID(), false)));
    }

    /**
     * Stops any control loops running on each side's motor controllers.
     */
    @Override
    public void finish() {
        leftMaster.stopMotor();
        rightMaster.stopMotor();
    }

    /**
     * Checks whether the left side's loop is currently on the target setpoint.
     *
     * @param controlMode the loop's control type (e.g. voltage, velocity, position...)
     * @param tolerance   the maximum difference from the left target to still consider the left loop to be on target
     * @param setpoint    the wanted setpoint
     * @return {@code true} when on target setpoint, {@code false} otherwise
     */
    @Override
    public boolean leftOnTarget(UnifiedControlMode controlMode, double tolerance, double setpoint) {
        double value = switch (controlMode) {
            case VELOCITY -> leftMaster.getVelocity().getValueAsDouble();
            case POSITION, MOTION_PROFILING, TRAPEZOID_PROFILE -> leftMaster.getPosition().getValueAsDouble();
            case CURRENT -> leftMaster.getTorqueCurrent().getValueAsDouble();
            case PERCENT_OUTPUT -> leftMaster.get();
            case VOLTAGE -> leftMaster.getMotorVoltage().getValueAsDouble();
        };
        return Math.abs(value - setpoint) <= tolerance;
    }

    /**
     * Checks whether the right side's loop is currently on the target setpoint.
     *
     * @param controlMode the loop's control type (e.g. voltage, velocity, position...)
     * @param tolerance   the maximum difference from the right target to still consider the right loop to be on target
     * @param setpoint    the wanted setpoint
     * @return {@code true} when on target setpoint, {@code false} otherwise
     */
    @Override
    public boolean rightOnTarget(UnifiedControlMode controlMode, double tolerance, double setpoint) {
        double value = switch (controlMode) {
            case VELOCITY -> rightMaster.getVelocity().getValueAsDouble();
            case POSITION, MOTION_PROFILING, TRAPEZOID_PROFILE -> rightMaster.getPosition().getValueAsDouble();
            case CURRENT -> rightMaster.getTorqueCurrent().getValueAsDouble();
            case PERCENT_OUTPUT -> rightMaster.get();
            case VOLTAGE -> rightMaster.getMotorVoltage().getValueAsDouble();
        };
        return Math.abs(value - setpoint) <= tolerance;
    }
}
