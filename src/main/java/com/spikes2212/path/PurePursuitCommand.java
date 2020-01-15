package com.spikes2212.path;

import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.PIDVASettings;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PurePursuitCommand extends CommandBase {

    private OdometryDrivetrain drivetrain;
    private Path path;
    private double lookaheadDistance;
    private PurePursuitController purePursuitController;
    private FeedForwardController rightFeedForwardController;
    private FeedForwardController leftFeedForwardController;
    private PIDVASettings leftSettings;
    private PIDVASettings rightSettings;
    private PIDController leftController;
    private PIDController rightController;

    public PurePursuitCommand(OdometryDrivetrain drivetrain, Path path, double lookaheadDistance,
                              PIDVASettings leftSettings, PIDVASettings rightSettings) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
        this.path = path;
        this.lookaheadDistance = lookaheadDistance;
        this.leftSettings = leftSettings;
        this.rightSettings = rightSettings;
    }

    @Override
    public void initialize() {
        drivetrain.resetSensors();
        purePursuitController = new PurePursuitController(drivetrain.getHandler(), path, lookaheadDistance,
                drivetrain.getWidth());
        purePursuitController.getHandler().set(purePursuitController.getPath().getPoints().get(0).getY(),
                purePursuitController.getPath().getPoints().get(0).getX());
        purePursuitController.reset();
        leftFeedForwardController = new FeedForwardController(leftSettings.getkV(), leftSettings.getkA(), 0.02);
        rightFeedForwardController = new FeedForwardController(rightSettings.getkV(), rightSettings.getkA(), 0.02);
        leftFeedForwardController.reset();
        rightFeedForwardController.reset();
        leftController = new PIDController(leftSettings.getkP(), leftSettings.getkI(), leftSettings.getkD());
        rightController = new PIDController(rightSettings.getkP(), rightSettings.getkI(), rightSettings.getkD());
        leftController.setTolerance(leftSettings.getTolerance());
        rightController.setTolerance(rightSettings.getTolerance());
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
        leftController.close();
        rightController.close();
    }

    @Override
    public boolean isFinished() {
        return purePursuitController.done();
    }
}
