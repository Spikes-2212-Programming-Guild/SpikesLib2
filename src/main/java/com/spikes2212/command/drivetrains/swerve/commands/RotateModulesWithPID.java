package com.spikes2212.command.drivetrains.swerve.commands;

import com.spikes2212.command.drivetrains.swerve.SwerveDrivetrain;
import com.spikes2212.command.drivetrains.swerve.SwerveModule;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A command that moves each {@link SwerveModule} in a given {@link SwerveDrivetrain} to a target angle
 * using closed-loop position control.
 *
 * @author Gil Ein-Gar
 * @see SwerveDrivetrain
 */
public class RotateModulesWithPID extends Command {

    private final SwerveDrivetrain drivetrain;

    private final Rotation2d frontLeftDesiredAngle;
    private final Rotation2d frontRightDesiredAngle;
    private final Rotation2d backLeftDesiredAngle;
    private final Rotation2d backRightDesiredAngle;

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private double lastTimeNotOnTarget;

    /**
     * Constructs a new {@link RotateModulesWithPID} command that moves the {@code SwerveModule}s of the
     * given {@code SwerveDrivetrain} to the desired {@link Rotation2d} angles.
     *
     * @param drivetrain             the swerve drivetrain this command operates on
     * @param frontLeftDesiredAngle  the desired front left angle
     * @param frontRightDesiredAngle the desired front right angle
     * @param backLeftDesiredAngle   the desired back left angle
     * @param backRightDesiredAngle  the desired back right angle
     */
    public RotateModulesWithPID(SwerveDrivetrain drivetrain, Rotation2d frontLeftDesiredAngle,
                                Rotation2d frontRightDesiredAngle, Rotation2d backLeftDesiredAngle,
                                Rotation2d backRightDesiredAngle) {
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
    }

    /**
     * Constructs a new {@code RotateModulesWithPID} command that moves the {@code SwerveModule}s of the
     * given {@code SwerveDrivetrain} to the desired angles in degrees.
     *
     * @param drivetrain             the swerve drivetrain this command operates on
     * @param frontLeftDesiredAngle  the desired front left angle in degrees
     * @param frontRightDesiredAngle the desired front right angle in degrees
     * @param backLeftDesiredAngle   the desired back left angle in degrees
     * @param backRightDesiredAngle  the desired back right angle in degrees
     */
    public RotateModulesWithPID(SwerveDrivetrain drivetrain, double frontLeftDesiredAngle,
                                double frontRightDesiredAngle, double backLeftDesiredAngle,
                                double backRightDesiredAngle) {
        this(drivetrain, Rotation2d.fromDegrees(frontLeftDesiredAngle),
                Rotation2d.fromDegrees(frontRightDesiredAngle), Rotation2d.fromDegrees(backLeftDesiredAngle),
                Rotation2d.fromDegrees(backRightDesiredAngle));
    }

    @Override
    public void initialize() {
        lastTimeNotOnTarget = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        frontLeftModule.setTargetAngle(frontLeftDesiredAngle);
        frontRightModule.setTargetAngle(frontRightDesiredAngle);
        backLeftModule.setTargetAngle(backLeftDesiredAngle);
        backRightModule.setTargetAngle(backRightDesiredAngle);
    }

    @Override
    public boolean isFinished() {
        if (!areModulesOnTarget()) lastTimeNotOnTarget = Timer.getFPGATimestamp();
        return Timer.getFPGATimestamp() - lastTimeNotOnTarget >= getMaxWaitTime();
    }

    private boolean areModulesOnTarget() {
        return isModuleOnTarget(frontLeftModule) && isModuleOnTarget(frontRightModule)
                && isModuleOnTarget(backLeftModule) && isModuleOnTarget(backRightModule);
    }

    private boolean isModuleOnTarget(SwerveModule module) {
        return module.getTurnMotor().onTarget(UnifiedControlMode.POSITION,
                module.getTurnMotorPIDSettings().getTolerance(),
                module.getTargetState().angle.getDegrees());
    }

    private double getMaxWaitTime() {
        double frontWaitTime = Math.max(frontLeftModule.getTurnMotorPIDSettings().getWaitTime(),
                frontRightModule.getTurnMotorPIDSettings().getWaitTime());
        double backWaitTime = Math.max(backLeftModule.getTurnMotorPIDSettings().getWaitTime(),
                backRightModule.getTurnMotorPIDSettings().getWaitTime());
        return Math.max(frontWaitTime, backWaitTime);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
