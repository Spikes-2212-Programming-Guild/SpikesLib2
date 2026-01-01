package com.spikes2212.command.drivetrains.swerve.commands;

import com.spikes2212.command.drivetrains.swerve.SwerveDrivetrain;
import com.spikes2212.command.drivetrains.swerve.SwerveModule;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A command that moves each {@link SwerveModule} in a given {@link SwerveDrivetrain} to a certain angle using pid to
 * position for each one of the {@link SwerveModule}s.
 *
 * @author Gil Ein-Gar
 * @see SwerveDrivetrain
 */
public class TurnModules extends Command {

    private static final double DEFAULT_WAIT_TIME = 0.2;

    private final SwerveDrivetrain drivetrain;

    private final Rotation2d frontLeftDesiredAngle;
    private final Rotation2d frontRightDesiredAngle;
    private final Rotation2d backLeftDesiredAngle;
    private final Rotation2d backRightDesiredAngle;

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private final boolean defaultWaitTime;
    private double lastTimeNotOnTarget;

    /**
     * Constructs a new {@link TurnModules} command that moves the given {@link SwerveDrivetrain}
     * {@link SwerveModule}s to a certain angle.
     *
     * @param drivetrain             the swerve drivetrain this command operates on
     * @param frontLeftDesiredAngle  the desired front left angle
     * @param frontRightDesiredAngle the desired front right angle
     * @param backLeftDesiredAngle   the desired back left angle
     * @param backRightDesiredAngle  the desired back right angle
     * @param defaultWaitTime        whether the modules will use the default wait time of the command
     */
    public TurnModules(SwerveDrivetrain drivetrain, Rotation2d frontLeftDesiredAngle,
                       Rotation2d frontRightDesiredAngle, Rotation2d backLeftDesiredAngle,
                       Rotation2d backRightDesiredAngle, boolean defaultWaitTime) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.frontLeftDesiredAngle = frontLeftDesiredAngle;
        this.frontRightDesiredAngle = frontRightDesiredAngle;
        this.backLeftDesiredAngle = backLeftDesiredAngle;
        this.backRightDesiredAngle = backRightDesiredAngle;

        this.frontLeftModule = drivetrain.getFrontLeftModule();
        this.frontRightModule = drivetrain.getFrontRightModule();
        this.backLeftModule = drivetrain.getBackLeftModule();
        this.backRightModule = drivetrain.getBackRightModule();

        this.defaultWaitTime = defaultWaitTime;

    }

    /**
     * Constructs a new {@link TurnModules} command that moves the given {@link SwerveDrivetrain}
     * {@link SwerveModule}s to a certain degree with a default wait time.
     *
     * @param drivetrain             the swerve drivetrain this command operates on
     * @param frontLeftDesiredAngle  the desired front left degree
     * @param frontRightDesiredAngle the desired front right degree
     * @param backLeftDesiredAngle   the desired back left degree
     * @param backRightDesiredAngle  the desired back right degree
     * @param defaultWaitTime        whether the modules will use the default wait time of the command
     */
    public TurnModules(SwerveDrivetrain drivetrain, double frontLeftDesiredAngle, double frontRightDesiredAngle,
                       double backLeftDesiredAngle, double backRightDesiredAngle, boolean defaultWaitTime) {
        this(drivetrain, Rotation2d.fromDegrees(frontLeftDesiredAngle), Rotation2d.fromDegrees(frontRightDesiredAngle),
                Rotation2d.fromDegrees(backLeftDesiredAngle), Rotation2d.fromDegrees(backRightDesiredAngle),
                defaultWaitTime);
    }

    /**
     * Moves each {@link SwerveModule} in the given {@link SwerveDrivetrain}
     * to the desired angle in degrees.
     */
    @Override
    public void execute() {
        frontLeftModule.setTargetAngle(frontLeftDesiredAngle);
        frontRightModule.setTargetAngle(frontRightDesiredAngle);
        backLeftModule.setTargetAngle(backLeftDesiredAngle);
        backRightModule.setTargetAngle(backRightDesiredAngle);
    }

    @Override
    public boolean isFinished() {
        if (!(frontLeftModule.getTurnMotor().onTarget(
                UnifiedControlMode.POSITION,
                frontLeftModule.getTurnMotorPIDSettings().getTolerance(),
                frontLeftDesiredAngle.getDegrees()) &&
                frontRightModule.getTurnMotor().onTarget(
                        UnifiedControlMode.POSITION,
                        frontRightModule.getTurnMotorPIDSettings().getTolerance(),
                        frontRightDesiredAngle.getDegrees()) &&
                backLeftModule.getTurnMotor().onTarget(
                        UnifiedControlMode.POSITION,
                        backLeftModule.getTurnMotorPIDSettings().getTolerance(),
                        backLeftDesiredAngle.getDegrees()) &&
                backRightModule.getTurnMotor().onTarget(
                        UnifiedControlMode.POSITION,
                        backRightModule.getTurnMotorPIDSettings().getTolerance(),
                        backRightDesiredAngle.getDegrees()))) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }

        if (!defaultWaitTime) {
            return Timer.getFPGATimestamp() -
                    lastTimeNotOnTarget >= DEFAULT_WAIT_TIME;
        }

        return Timer.getFPGATimestamp() -
                lastTimeNotOnTarget >= frontLeftModule.getTurnMotorPIDSettings().getWaitTime();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
