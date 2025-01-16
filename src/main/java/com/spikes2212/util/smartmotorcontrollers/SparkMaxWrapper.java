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

    //@TODO change ffcontroller param to relevant enum constant
    public SparkMaxWrapper(int deviceID, MotorType type, FeedForwardController feedForwardController) {
        super(deviceID, type);
        this.feedForwardController = feedForwardController;
    }

    public SparkMaxWrapper(int deviceID, MotorType type) {
        this(deviceID, type, new FeedForwardController(FeedForwardSettings.EMPTY_FFSETTINGS,
                FeedForwardController.DEFAULT_PERIOD));
    }

    public void setInverted(boolean inverted) {
        configure(new SparkMaxConfig().inverted(inverted), ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    public void follow(SparkBase master) {
        configure(new SparkMaxConfig().follow(master), ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    @Override
    public void configurePID(PIDSettings pidSettings) {
        ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
        closedLoopConfig.pid(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        configure(new SparkMaxConfig().apply(closedLoopConfig), ResetMode.kNoResetSafeParameters,
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
