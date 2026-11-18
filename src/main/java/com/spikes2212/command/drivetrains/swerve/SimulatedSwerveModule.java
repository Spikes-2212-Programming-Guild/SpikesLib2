package com.spikes2212.command.drivetrains.swerve;

import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.util.Simulatable;
import com.spikes2212.util.smartmotorcontrollers.SimulatedSmartMotorControllerWrapper;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * A simulated {@link SwerveModule} that runs closed-loop steering and driving physics using
 * {@link SimulatedSmartMotorControllerWrapper} instances.
 *
 * @author Adi Yekutiel
 * @see SwerveModule
 * @see Simulatable
 */
public class SimulatedSwerveModule extends SwerveModule implements Simulatable {

    private static final double DEFAULT_WHEEL_RADIUS_METERS = 0.0508;
    private static final double DEFAULT_DRIVE_GEAR_RATIO = 6.75;
    private static final double DEFAULT_TURN_GEAR_RATIO = 150.0 / 7.0;
    private static final double DEFAULT_DRIVE_MOI = 0.025;
    private static final double DEFAULT_TURN_MOI = 0.004;
    private static final double DEFAULT_MIN_SPEED_LIMIT = 0.05;
    private static final double DEFAULT_ABSOLUTE_OFFSET = 0.0;
    private static final double SECONDS_PER_MINUTE = 60.0;
    private static final double CIRCUMFERENCE_MULTIPLIER = 2.0 * Math.PI;
    private static final int SINGLE_MOTOR = 1;

    protected final SimulatedSmartMotorControllerWrapper simulatedDriveMotor;
    protected final SimulatedSmartMotorControllerWrapper simulatedTurnMotor;

    private final double wheelRadiusMeters;

    public SimulatedSwerveModule(String namespaceName,
                                 SimulatedSmartMotorControllerWrapper driveMotor,
                                 SimulatedSmartMotorControllerWrapper turnMotor,
                                 boolean driveMotorInverted,
                                 boolean turnMotorInverted,
                                 double absoluteEncoderOffset,
                                 PIDSettings driveMotorPIDSettings,
                                 PIDSettings turnMotorPIDSettings,
                                 FeedForwardSettings driveMotorFFSettings,
                                 FeedForwardSettings turnMotorFFSettings,
                                 double minSpeedLimit,
                                 boolean updateDrivePeriodically,
                                 boolean updateTurnPeriodically,
                                 double wheelRadiusMeters) {
        super(
                namespaceName,
                driveMotor,
                turnMotor,
                driveMotorInverted,
                turnMotorInverted,
                absoluteEncoderOffset,
                driveMotorPIDSettings,
                turnMotorPIDSettings,
                driveMotorFFSettings,
                turnMotorFFSettings,
                minSpeedLimit,
                updateDrivePeriodically,
                updateTurnPeriodically
        );
        this.simulatedDriveMotor = driveMotor;
        this.simulatedTurnMotor = turnMotor;
        this.wheelRadiusMeters = wheelRadiusMeters;

        configureDriveController();
        configureTurnController();
        configureAbsoluteEncoder();
    }

    public SimulatedSwerveModule(String namespaceName,
                                 SimulatedSmartMotorControllerWrapper driveMotor,
                                 SimulatedSmartMotorControllerWrapper turnMotor,
                                 PIDSettings driveMotorPIDSettings,
                                 PIDSettings turnMotorPIDSettings,
                                 FeedForwardSettings driveMotorFFSettings,
                                 FeedForwardSettings turnMotorFFSettings,
                                 double wheelRadiusMeters) {
        this(
                namespaceName,
                driveMotor,
                turnMotor,
                false,
                false,
                DEFAULT_ABSOLUTE_OFFSET,
                driveMotorPIDSettings,
                turnMotorPIDSettings,
                driveMotorFFSettings,
                turnMotorFFSettings,
                DEFAULT_MIN_SPEED_LIMIT,
                true,
                true,
                wheelRadiusMeters
        );
    }

    public SimulatedSwerveModule(String namespaceName,
                                 DCMotor driveMotorType,
                                 double driveGearRatio,
                                 double driveMoi,
                                 DCMotor turnMotorType,
                                 double turnGearRatio,
                                 double turnMoi,
                                 PIDSettings driveMotorPIDSettings,
                                 PIDSettings turnMotorPIDSettings,
                                 FeedForwardSettings driveMotorFFSettings,
                                 FeedForwardSettings turnMotorFFSettings,
                                 double wheelRadiusMeters) {
        this(
                namespaceName,
                new SimulatedSmartMotorControllerWrapper(driveMotorType, driveGearRatio, driveMoi),
                new SimulatedSmartMotorControllerWrapper(turnMotorType, turnGearRatio, turnMoi),
                driveMotorPIDSettings,
                turnMotorPIDSettings,
                driveMotorFFSettings,
                turnMotorFFSettings,
                wheelRadiusMeters
        );
    }

    public SimulatedSwerveModule(String namespaceName,
                                 PIDSettings driveMotorPIDSettings,
                                 PIDSettings turnMotorPIDSettings,
                                 FeedForwardSettings driveMotorFFSettings,
                                 FeedForwardSettings turnMotorFFSettings) {
        this(
                namespaceName,
                DCMotor.getNEO(SINGLE_MOTOR),
                DEFAULT_DRIVE_GEAR_RATIO,
                DEFAULT_DRIVE_MOI,
                DCMotor.getNEO(SINGLE_MOTOR),
                DEFAULT_TURN_GEAR_RATIO,
                DEFAULT_TURN_MOI,
                driveMotorPIDSettings,
                turnMotorPIDSettings,
                driveMotorFFSettings,
                turnMotorFFSettings,
                DEFAULT_WHEEL_RADIUS_METERS
        );
    }

    @Override
    protected void configureDriveController() {
        double wheelCircumferenceMeters = CIRCUMFERENCE_MULTIPLIER * wheelRadiusMeters;
        simulatedDriveMotor.setInverted(driveMotorInverted);
        simulatedDriveMotor.setPositionConversionFactor(wheelCircumferenceMeters);
        simulatedDriveMotor.setVelocityConversionFactor(wheelCircumferenceMeters / SECONDS_PER_MINUTE);
        simulatedDriveMotor.configurePID(driveMotorPIDSettings);
        simulatedDriveMotor.configureFF(driveMotorFFSettings);
    }

    @Override
    protected void configureTurnController() {
        simulatedTurnMotor.setInverted(turnMotorInverted);
        simulatedTurnMotor.setPositionConversionFactor(DEGREES_IN_ROTATIONS);
        simulatedTurnMotor.setVelocityConversionFactor(DEGREES_IN_ROTATIONS / SECONDS_PER_MINUTE);
        simulatedTurnMotor.configurePID(turnMotorPIDSettings);
        simulatedTurnMotor.configureFF(turnMotorFFSettings);
    }

    @Override
    protected void configureAbsoluteEncoder() {
        simulatedTurnMotor.setPosition(absoluteEncoderOffset);
    }

    @Override
    protected Rotation2d getAbsoluteModuleAngle() {
        return Rotation2d.fromDegrees(getRelativeModuleAngle());
    }

    @Override
    public void update(double dtSeconds) {
        simulatedDriveMotor.update(dtSeconds);
        simulatedTurnMotor.update(dtSeconds);
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("angle", this::getRelativeModuleAngle);
        namespace.putNumber("velocity", this::getDrivingVelocity);
    }
}
