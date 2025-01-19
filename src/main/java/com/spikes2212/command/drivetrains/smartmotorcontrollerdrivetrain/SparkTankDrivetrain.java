package com.spikes2212.command.drivetrains.smartmotorcontrollerdrivetrain;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.*;
import com.spikes2212.command.drivetrains.TankDrivetrain;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import java.util.List;

/**
 * A {@link TankDrivetrain}, whose sides each consists of a master {@link SparkBase} motor controller that runs
 * control loops and additional {@link SparkBase} motor controllers that follow it.
 *
 * @author Yoel Perman Brilliant
 * @see TankDrivetrain
 * @see SmartMotorControllerTankDrivetrain
 */
public class SparkTankDrivetrain extends TankDrivetrain implements SmartMotorControllerTankDrivetrain {

    /**
     * The slot on the motor controller on which the trapezoid profiling configurations are saved.
     */
    private static final int TRAPEZOID_SLOT_ID = 0;

    /**
     * The left motor controller which runs the loops.
     */
    protected final SparkBase leftMaster;

    /**
     * The right motor controller which runs the loops.
     */
    protected final SparkBase rightMaster;

    /**
     * Additional motor controllers that follow the left master.
     */
    protected final List<? extends SparkBase> leftSlaves;

    /**
     * Additional motor controllers that follow the right master.
     */
    protected final List<? extends SparkBase> rightSlaves;

    protected final SparkBaseConfig leftConfig;
    protected final SparkBaseConfig rightConfig;

    /**
     * Constructs a new instance of {@link SparkTankDrivetrain}.
     *
     * @param namespaceName the name of the drivetrain's namespace
     * @param leftMaster    the {@link SparkBase} motor controller which runs the left side's loops
     * @param leftSlaves    additional {@link SparkBase} motor controllers that follow the left master
     * @param rightMaster   the {@link SparkBase} motor controller which runs the right side's loops
     * @param rightSlaves   additional {@link SparkBase} motor controllers that follow the right master
     */
    public SparkTankDrivetrain(String namespaceName, SparkBase leftMaster, List<? extends SparkBase> leftSlaves,
                               SparkBase rightMaster, List<? extends SparkBase> rightSlaves) {
        super(namespaceName, leftMaster, rightMaster);
        this.leftMaster = leftMaster;
        this.rightMaster = rightMaster;
        this.leftSlaves = leftSlaves;
        this.rightSlaves = rightSlaves;
        if (leftMaster instanceof SparkMax) {
            leftConfig = new SparkMaxConfig();
            rightConfig = new SparkMaxConfig();
        }
        else if (leftMaster instanceof SparkFlex) {
            leftConfig = new SparkFlexConfig();
            rightConfig = new SparkFlexConfig();
        }
        else {
            leftConfig = null;
            rightConfig = null;
        }
        this.leftSlaves.forEach(s -> {
            s.configure(leftConfig.follow(leftMaster), SparkBase.ResetMode.kNoResetSafeParameters,
                    SparkBase.PersistMode.kNoPersistParameters);
        });

        rightConfig.inverted(true);
        this.rightSlaves.forEach(s -> {
            s.configure(rightConfig.follow(rightMaster), SparkBase.ResetMode.kNoResetSafeParameters,
                    SparkBase.PersistMode.kNoPersistParameters);
        });
    }

    /**
     * Constructs a new instance of {@link SparkTankDrivetrain}.
     *
     * @param namespaceName the name of the drivetrain's namespace
     * @param leftMaster    the {@link SparkBase} motor controller which runs the left side's loops
     * @param leftSlave     an additional {@link SparkBase} motor controller that follows the left master
     * @param rightMaster   the {@link SparkBase} motor controller which runs the right side's loops
     * @param rightSlave    an additional {@link SparkBase} motor controller that follows the right master
     */
    public SparkTankDrivetrain(String namespaceName, SparkBase leftMaster, SparkBase leftSlave,
                               SparkBase rightMaster, SparkBase rightSlave) {
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
        ClosedLoopConfig leftClosedLoopConfig = new ClosedLoopConfig();
        leftClosedLoopConfig.pidf(leftPIDSettings.getkP(), leftPIDSettings.getkI(), leftPIDSettings.getkD(),
                feedForwardSettings.getkV());
        leftConfig.apply(leftClosedLoopConfig);
        leftMaster.configure(leftConfig, SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);

        ClosedLoopConfig rightClosedLoopConfig = new ClosedLoopConfig();
        rightClosedLoopConfig.pidf(rightPIDSettings.getkP(), rightPIDSettings.getkI(), rightPIDSettings.getkD(),
                feedForwardSettings.getkV());
        rightConfig.apply(rightClosedLoopConfig);
        rightMaster.configure(rightConfig, SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);
    }

    /**
     * Configures the loops' trapezoid profile settings.
     */
    @Override
    public void configureTrapezoid(TrapezoidProfileSettings settings) {
        MAXMotionConfig leftMaxMotionConfig = new MAXMotionConfig();
        leftMaxMotionConfig.maxAcceleration(settings.getMaxAcceleration());
        leftMaxMotionConfig.maxVelocity(settings.getMaxAcceleration());
        // @TODO what
//        master.getPIDController().setSmartMotionAccelStrategy(
//                SparkPIDController.AccelStrategy.fromInt((int) settings.getCurve()), TRAPEZOID_SLOT_ID);


        MAXMotionConfig rightMaxMotionConfig = new MAXMotionConfig();
        rightMaxMotionConfig.maxAcceleration(settings.getMaxAcceleration());
        rightMaxMotionConfig.maxVelocity(settings.getMaxAcceleration());
//        master.getPIDController().setSmartMotionAccelStrategy(
//                SparkPIDController.AccelStrategy.fromInt((int) settings.getCurve()), TRAPEZOID_SLOT_ID);
    }

    /**
     * Configures the loops' settings.
     */
    @Override
    public void configureLoop(PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                              FeedForwardSettings feedForwardSettings,
                              TrapezoidProfileSettings trapezoidProfileSettings) {
        leftMaster.configure(leftConfig, SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);
        leftSlaves.forEach(s -> s.configure(leftConfig, SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters));

        rightConfig.inverted(true);
        rightMaster.configure(rightConfig, SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);
        rightSlaves.forEach(s -> s.configure(rightConfig, SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters));

        configPIDF(leftPIDSettings, rightPIDSettings, feedForwardSettings);
        configureTrapezoid(trapezoidProfileSettings);
    }

    /**
     * Updates any control loops running on the master {@link SparkBase} motor controllers.
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

        leftMaster.getClosedLoopController().setReference(leftSetpoint, controlMode.getSparkMaxControlType());
        rightMaster.getClosedLoopController().setReference(rightSetpoint, controlMode.getSparkMaxControlType());
    }

    /**
     * Stops any control loops running on each side's master motor controller.
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
            case PERCENT_OUTPUT -> leftMaster.getAppliedOutput();
            case VELOCITY -> leftMaster.getEncoder().getVelocity();
            case CURRENT -> leftMaster.getOutputCurrent();
            case VOLTAGE -> leftMaster.getBusVoltage() * leftMaster.getAppliedOutput();
            default -> leftMaster.getEncoder().getPosition();
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
            case PERCENT_OUTPUT -> rightMaster.getAppliedOutput();
            case VELOCITY -> rightMaster.getEncoder().getVelocity();
            case CURRENT -> rightMaster.getOutputCurrent();
            case VOLTAGE -> rightMaster.getBusVoltage() * rightMaster.getAppliedOutput();
            default -> rightMaster.getEncoder().getPosition();
        };
        return Math.abs(value - setpoint) <= tolerance;
    }
}
