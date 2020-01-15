package com.spikes2212.command.drivetrains.commands;

import com.spikes2212.command.drivetrains.OdometryDrivetrain;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.PIDVASettings;
import com.spikes2212.path.Path;
import com.spikes2212.path.PurePursuitController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FollowPath extends CommandBase {

    private OdometryDrivetrain drivetrain;
    private Path path;
    private double lookaheadDistance;
    private PurePursuitController purePursuitController;
    private FeedForwardController rightFeedForwardController;
    private FeedForwardController leftFeedForwardController;
    private PIDVASettings pidvaSettings;
    private PIDController leftController;
    private PIDController rightController;

    public FollowPath(OdometryDrivetrain drivetrain, Path path, double lookaheadDistance,
                      PIDVASettings pidvaSettings) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.path = path;
        this.lookaheadDistance = lookaheadDistance;
        this.pidvaSettings = pidvaSettings;
    }

    @Override
    public void initialize() {
        drivetrain.zeroSensors();
        purePursuitController = new PurePursuitController(drivetrain.getHandler(), path, lookaheadDistance,
                drivetrain.getWidth());
        purePursuitController.getOdometryHandler().set(purePursuitController.getPath().getPoints().get(0).getY(),
                purePursuitController.getPath().getPoints().get(0).getX());
        purePursuitController.reset();
        leftFeedForwardController = new FeedForwardController(pidvaSettings.getkV(), pidvaSettings.getkA(), 0.02);
        rightFeedForwardController = new FeedForwardController(pidvaSettings.getkV(), pidvaSettings.getkA(), 0.02);
        leftFeedForwardController.reset();
        rightFeedForwardController.reset();
        leftController = new PIDController(pidvaSettings.getkP(), pidvaSettings.getkI(), pidvaSettings.getkD());
        rightController = new PIDController(pidvaSettings.getkP(), pidvaSettings.getkI(), pidvaSettings.getkD());
    }

    @Override
    public void execute() {
        double[] speeds = purePursuitController.getTargetSpeeds();
        double leftSpeed = leftFeedForwardController.calculate(speeds[0]) + leftController.calculate(
                drivetrain.getLeftRate(), speeds[0]
        );
        double rightSpeed = rightFeedForwardController.calculate(speeds[1]) + rightController.calculate(
                drivetrain.getRightRate(), speeds[1]
        );
        drivetrain.tankDrive(leftSpeed, rightSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return purePursuitController.done();
    }
}
