package com.spikes2212.command.drivetrains.swerve;

import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.util.UnifiedControlMode;
import com.spikes2212.util.smartmotorcontrollers.SmartMotorController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static java.lang.Math.abs;

/**
 * A single swerve module, serving as the abstraction for one wheel's steering and driving
 * mechanisms within a swerve drivetrain.
 *
 * @author Gil Ein-Gar
 * @see DashboardedSubsystem
 */
public abstract class SwerveModule extends DashboardedSubsystem {

    protected static final double DEGREES_IN_ROTATIONS = 360;
    private static final double DEGREES_TO_FLIP = 180;
    private static final double MAX_DISTANCE_TO_ROTATE = 90;

    protected final boolean driveMotorInverted;
    protected final boolean turnMotorInverted;
    protected final double minSpeedLimit;
    protected final boolean updateDrivePeriodically;
    protected final boolean updateTurnPeriodically;

    private final SmartMotorController driveMotor;
    private final SmartMotorController turnMotor;

    protected double absoluteEncoderOffset;
    protected PIDSettings driveMotorPIDSettings;
    protected PIDSettings turnMotorPIDSettings;
    protected FeedForwardSettings driveMotorFFSettings;
    protected FeedForwardSettings turnMotorFFSettings;
    protected SwerveModuleState targetState = new SwerveModuleState();

    /**
     * Constructs a new instance of {@link SwerveModule}.
     *
     * @param namespaceName           the namespace name used for the module
     * @param driveMotor              the motor used for driving the wheel
     * @param turnMotor               the motor used for steering the wheel
     * @param driveMotorInverted      whether the drive motor is inverted
     * @param turnMotorInverted       whether the turn motor is inverted
     * @param absoluteEncoderOffset   the absolute encoder offset when the wheel is parallel to the track
     * @param driveMotorPIDSettings   the drive motor PID settings
     * @param turnMotorPIDSettings    the turn motor PID settings
     * @param driveMotorFFSettings    the drive motor feed forward settings
     * @param turnMotorFFSettings     the turn motor feed forward settings
     * @param minSpeedLimit           a minimum speed threshold intended for dealing with stick drift
     * @param updateDrivePeriodically whether to update the drive controller gains periodically
     * @param updateTurnPeriodically  whether to update the turn controller gains periodically
     */
    public SwerveModule(String namespaceName, SmartMotorController driveMotor, SmartMotorController turnMotor,
                        boolean driveMotorInverted, boolean turnMotorInverted, double absoluteEncoderOffset,
                        PIDSettings driveMotorPIDSettings, PIDSettings turnMotorPIDSettings,
                        FeedForwardSettings driveMotorFFSettings, FeedForwardSettings turnMotorFFSettings, double minSpeedLimit,
                        boolean updateDrivePeriodically, boolean updateTurnPeriodically) {
        super(namespaceName);
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.driveMotorInverted = driveMotorInverted;
        this.turnMotorInverted = turnMotorInverted;
        this.absoluteEncoderOffset = absoluteEncoderOffset;
        this.driveMotorPIDSettings = driveMotorPIDSettings;
        this.turnMotorPIDSettings = turnMotorPIDSettings;
        this.driveMotorFFSettings = driveMotorFFSettings;
        this.turnMotorFFSettings = turnMotorFFSettings;
        this.minSpeedLimit = minSpeedLimit;
        this.updateDrivePeriodically = updateDrivePeriodically;
        this.updateTurnPeriodically = updateTurnPeriodically;
    }

    /**
     * Resets the relative encoder according to the absolute encoder.
     */
    public void resetRelativeEncoder() {
        turnMotor.setPosition(getAbsoluteModuleAngle().getDegrees());
    }

    /**
     * Stops the drive and turn motors of this {@code SwerveModule}.
     */
    public void stop() {
        driveMotor.stopMotor();
        turnMotor.stopMotor();
    }

    /**
     * Configures drive motor conversion factors that translate encoder rotations
     * into real-world distance and speed measurements.
     */
    protected abstract void configureDriveController();

    /**
     * Configures turn motor conversion factors that convert motor rotations
     * into angular position and velocity values in degrees.
     */
    protected abstract void configureTurnController();

    /**
     * Configures the absolute encoder direction, offset, and calibration parameters
     * to align encoder readings with the module mechanics.
     */
    protected abstract void configureAbsoluteEncoder();

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(getDrivingVelocity(), Rotation2d.fromDegrees(getRelativeModuleAngle()));
    }

    public SwerveModuleState getTargetState() {
        return targetState;
    }

    /**
     * Sets the desired state of this {@code SwerveModule}.
     *
     * @param targetState the desired state of the module
     */
    public void setTargetState(SwerveModuleState targetState) {
        if (abs(targetState.speedMetersPerSecond) <= minSpeedLimit) {
            this.targetState.speedMetersPerSecond = 0;
            stop();
            return;
        }
        SwerveModuleState optimizedState = optimize(targetState, getRelativeModuleAngle());
        setTargetAngle(optimizedState.angle);
        setTargetVelocity(optimizedState.speedMetersPerSecond);
    }

    /**
     * Optimizes the desired {@link SwerveModuleState} to minimize steering rotation.
     *
     * @param targetState  the desired state of this {@code SwerveModule}
     * @param currentAngle the current angle of this {@code SwerveModule} in degrees
     * @return the optimized {@code SwerveModuleState}
     */
    private SwerveModuleState optimize(SwerveModuleState targetState, double currentAngle) {
        double desiredAngle = normalizeAngleRelativeToRelativeEncoder(currentAngle, targetState.angle.getDegrees());
        double targetSpeed = targetState.speedMetersPerSecond;
        if (abs(desiredAngle - currentAngle) > MAX_DISTANCE_TO_ROTATE) {
            desiredAngle -= Math.copySign(DEGREES_TO_FLIP, desiredAngle - currentAngle);
            targetSpeed = -targetSpeed;
        }
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(desiredAngle));
    }

    /**
     * Normalizes the desired angle to be within [-180, 180] degrees of the current relative encoder angle.
     *
     * @param currentAngle the current angle of this {@code SwerveModule} in degrees
     * @param desiredAngle the desired angle of this {@code SwerveModule} in degrees
     * @return the normalized desired angle in degrees
     */
    private double normalizeAngleRelativeToRelativeEncoder(double currentAngle, double desiredAngle) {
        return currentAngle + Math.IEEEremainder(desiredAngle - currentAngle, DEGREES_IN_ROTATIONS);
    }

    /**
     * Sets the desired angle of this {@code SwerveModule}.
     *
     * @param targetAngle the desired angle of the module
     */
    public void setTargetAngle(Rotation2d targetAngle) {
        double normalizedAngle =
                normalizeAngleRelativeToRelativeEncoder(getRelativeModuleAngle(), targetAngle.getDegrees());
        targetState.angle = Rotation2d.fromDegrees(normalizedAngle);
        turnMotor.pidSet(UnifiedControlMode.POSITION, normalizedAngle, turnMotorPIDSettings,
                turnMotorFFSettings, updateTurnPeriodically);
    }

    /**
     * Sets the desired velocity of this {@code SwerveModule}.
     *
     * @param targetVelocity the desired velocity of the module in meters per second
     */
    public void setTargetVelocity(double targetVelocity) {
        targetState.speedMetersPerSecond = targetVelocity;
        driveMotor.pidSet(UnifiedControlMode.VELOCITY, targetVelocity, driveMotorPIDSettings,
                driveMotorFFSettings, updateDrivePeriodically);
    }

    public double getRelativeModuleAngle() {
        return turnMotor.getPosition();
    }

    /**
     * Gets the current module velocity.
     *
     * @return the current module velocity in meters per second
     */
    public double getDrivingVelocity() {
        return driveMotor.getVelocity();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getPosition(), getAbsoluteModuleAngle());
    }

    public PIDSettings getDriveMotorPIDSettings() {
        return driveMotorPIDSettings;
    }

    public PIDSettings getTurnMotorPIDSettings() {
        return turnMotorPIDSettings;
    }

    public FeedForwardSettings getDriveMotorFeedForwardSettings() {
        return driveMotorFFSettings;
    }

    public FeedForwardSettings getTurnMotorFeedForwardSettings() {
        return turnMotorFFSettings;
    }

    public SmartMotorController getTurnMotor() {
        return turnMotor;
    }

    public SmartMotorController getDriveMotor() {
        return driveMotor;
    }

    /**
     * Gets the absolute module angle using an absolute encoder.
     *
     * @return the absolute module angle
     */
    protected abstract Rotation2d getAbsoluteModuleAngle();
}
