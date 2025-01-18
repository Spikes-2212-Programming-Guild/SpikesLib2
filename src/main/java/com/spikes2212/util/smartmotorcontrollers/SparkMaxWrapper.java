package com.spikes2212.util.smartmotorcontrollers;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.UnifiedControlMode;

public class SparkMaxWrapper extends SparkMax implements SmartMotorController {

    private final FeedForwardController feedForwardController;
    private SparkMaxConfig sparkConfig;
    private ClosedLoopConfig closedLoopConfig;

    //@TODO change ffcontroller param to relevant enum constant
    public SparkMaxWrapper(int deviceID, MotorType type, FeedForwardController feedForwardController) {
        super(deviceID, type);
        this.feedForwardController = feedForwardController;
        sparkConfig = new SparkMaxConfig();
        closedLoopConfig = new ClosedLoopConfig();
    }

    public SparkMaxWrapper(int deviceID, MotorType type) {
        this(deviceID, type, new FeedForwardController(FeedForwardSettings.EMPTY_FFSETTINGS,
                FeedForwardController.DEFAULT_PERIOD));
    }

    public void restoreFactoryDefaults() {
        sparkConfig = new SparkMaxConfig();
        closedLoopConfig = new ClosedLoopConfig();
        configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setInverted(boolean inverted) {
        configure(sparkConfig.inverted(inverted), ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    public void follow(SparkBase master) {
        configure(sparkConfig.follow(master), ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    @Override
    public void configurePID(PIDSettings pidSettings) {
        closedLoopConfig.pid(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        configure(sparkConfig.apply(closedLoopConfig), ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    @Override
    public void configureFF(FeedForwardSettings feedForwardSettings) {
        feedForwardController.setGains(feedForwardSettings);
    }

    @Override
    public void configureTrapezoid(TrapezoidProfileSettings trapezoidProfileSettings) {
        // @TODO figure out trapezoids
    }

    @Override
    public void pidSet(UnifiedControlMode controlMode, double setpoint, PIDSettings pidSettings,
                       FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings) {
        configureLoop(pidSettings, feedForwardSettings, trapezoidProfileSettings);
        getClosedLoopController().setReference(setpoint, controlMode.getSparkMaxControlType(), ClosedLoopSlot.kSlot0,
                feedForwardController.calculate(setpoint));
    }
}
