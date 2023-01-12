package com.spikes2212.command.drivetrains;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Collection;
import java.util.List;

/**
 * A {@link TankDrivetrain}, whose sides each consists of a master {@link CANSparkMax} motor controller that runs control
 * loops and additional {@link CANSparkMax} motor controllers that follow it.
 *
 * @author Yoel Perman Brilliant
 * @see DashboardedSubsystem
 * @see SmartMotorControllerTankDrivetrain
 */
public class SparkMaxDrivetrain extends TankDrivetrain implements SmartMotorControllerTankDrivetrain {

    /**
     * The slot on the {@link CANSparkMax} on which the trapezoid profiling configurations are saved.
     */
    private static final int TRAPEZOID_SLOT_ID = 0;

    /**
     * The left {@link CANSparkMax} which run the loops.
     */
    protected final CANSparkMax leftMaster;

    /**
     * The right {@link CANSparkMax} which run the loops.
     */
    protected final CANSparkMax rightMaster;

    /**
     * Additional {@link CANSparkMax}s that follow the left master.
     */
    protected final List<CANSparkMax> leftSlaves;

    /**
     * Additional {@link CANSparkMax}s that follow the right master.
     */
    protected final List<CANSparkMax> rightSlaves;

    /**
     * Constructs a new instance of {@link SparkMaxDrivetrain}.
     *
     * @param namespaceName the name of the subsystem's namespace
     * @param leftMaster    the motor controller which runs the left side's loops
     * @param leftSlaves    additional motor controllers that follow the left master
     * @param rightMaster   the motor controller which runs the right side's loops
     * @param rightSlaves   additional motor controllers that follow the right master
     */
    public SparkMaxDrivetrain(String namespaceName, CANSparkMax leftMaster, CANSparkMax rightMaster,
                              List<CANSparkMax> leftSlaves, List<CANSparkMax> rightSlaves) {
        super(namespaceName, leftMaster, rightMaster);
        this.leftMaster = leftMaster;
        this.rightMaster = rightMaster;
        this.leftSlaves = leftSlaves;
        this.rightSlaves = rightSlaves;
        this.leftSlaves.forEach(s -> s.follow(leftMaster));
        this.rightSlaves.forEach(s -> s.follow(rightMaster));
    }

    /**
     * Adds any data or commands to the {@link NetworkTable}s, which can be accessed using the {@link Shuffleboard}.
     */
    @Override
    public void configureDashboard() {
    }

    /**
     * Configures the loop's PID constants and feed forward gains.
     */
    @Override
    public void configPIDF(PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                           FeedForwardSettings feedForwardSettings) {
        leftMaster.getPIDController().setFF(feedForwardSettings.getkV());
        leftMaster.getPIDController().setP(leftPIDSettings.getkP());
        leftMaster.getPIDController().setI(leftPIDSettings.getkI());
        leftMaster.getPIDController().setD(leftPIDSettings.getkD());
        rightMaster.getPIDController().setFF(feedForwardSettings.getkV());
        rightMaster.getPIDController().setP(rightPIDSettings.getkP());
        rightMaster.getPIDController().setI(rightPIDSettings.getkI());
        rightMaster.getPIDController().setD(rightPIDSettings.getkD());
    }

    /**
     * Configures the loop's trapezoid profile settings.
     */
    @Override
    public void configureTrapezoid(TrapezoidProfileSettings settings) {
        leftMaster.getPIDController().setSmartMotionMaxAccel(settings.getAccelerationRate(), TRAPEZOID_SLOT_ID);
        leftMaster.getPIDController().setSmartMotionMaxVelocity(settings.getMaxVelocity(), TRAPEZOID_SLOT_ID);
        leftMaster.getPIDController().setSmartMotionAccelStrategy(
                SparkMaxPIDController.AccelStrategy.fromInt(settings.getCurve()), TRAPEZOID_SLOT_ID);
        rightMaster.getPIDController().setSmartMotionMaxAccel(settings.getAccelerationRate(), TRAPEZOID_SLOT_ID);
        rightMaster.getPIDController().setSmartMotionMaxVelocity(settings.getMaxVelocity(), TRAPEZOID_SLOT_ID);
        rightMaster.getPIDController().setSmartMotionAccelStrategy(
                SparkMaxPIDController.AccelStrategy.fromInt(settings.getCurve()), TRAPEZOID_SLOT_ID);
    }

    /**
     * Configures the loop's settings.
     */
    @Override
    public void configureLoop(PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                              FeedForwardSettings feedForwardSettings,
                              TrapezoidProfileSettings trapezoidProfileSettings) {
        leftMaster.restoreFactoryDefaults();
        rightMaster.restoreFactoryDefaults();
        configPIDF(leftPIDSettings, rightPIDSettings, feedForwardSettings);
        configureTrapezoid(trapezoidProfileSettings);
    }

    /**
     * Updates any control loops running on the motor controller.
     *
     * @param controlMode              the loop's control type (e.g. voltage, velocity, position...)
     * @param leftSetpoint             the left side loop's target setpoint
     * @param rightSetpoint            the right side loop's target setpoint
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
        configureTrapezoid(trapezoidProfileSettings);
        leftMaster.getPIDController().setReference(leftSetpoint, controlMode.getSparkMaxControlType());
        rightMaster.getPIDController().setReference(rightSetpoint, controlMode.getSparkMaxControlType());
    }

    /**
     * Stops any control loops running on the motor controller.
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
     * @param tolerance   the maximum difference from the target to still be considered on target
     * @param setpoint    the wanted setpoint
     * @return {@code true} when on target setpoint, {@code false} otherwise
     */
    @Override
    public boolean leftOnTarget(UnifiedControlMode controlMode, double tolerance, double setpoint) {
        double value;
        switch (controlMode) {
            case PERCENT_OUTPUT:
                value = leftMaster.getAppliedOutput();
                break;
            case VELOCITY:
                value = leftMaster.getEncoder().getVelocity();
                break;
            case CURRENT:
                value = leftMaster.getOutputCurrent();
                break;
            case VOLTAGE:
                value = leftMaster.getBusVoltage();
                break;
            default:
                value = leftMaster.getEncoder().getPosition();
        }
        return Math.abs(value - setpoint) <= tolerance;
    }

    /**
     * Checks whether the right side's loop is currently on the target setpoint.
     *
     * @param controlMode the loop's control type (e.g. voltage, velocity, position...)
     * @param tolerance   the maximum difference from the target to still be considered on target
     * @param setpoint    the wanted setpoint
     * @return {@code true} when on target setpoint, {@code false} otherwise
     */
    @Override
    public boolean rightOnTarget(UnifiedControlMode controlMode, double tolerance, double setpoint) {
        double value;
        switch (controlMode) {
            case PERCENT_OUTPUT:
                value = rightMaster.getAppliedOutput();
                break;
            case VELOCITY:
                value = rightMaster.getEncoder().getVelocity();
                break;
            case CURRENT:
                value = rightMaster.getOutputCurrent();
                break;
            case VOLTAGE:
                value = rightMaster.getBusVoltage();
                break;
            default:
                value = rightMaster.getEncoder().getPosition();
        }
        return Math.abs(value - setpoint) <= tolerance;
    }
}
