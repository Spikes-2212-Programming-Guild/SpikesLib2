package com.spikes2212.util;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;

public class SparkBaseWrapper extends SparkBase {

    protected final SparkBaseConfig sparkConfig;
    protected final ClosedLoopConfig closedLoopConfig;

    protected SparkBaseWrapper(int deviceId, MotorType type, SparkModel model) {
        super(deviceId, type, model);
        switch (model) {
            case SparkMax -> sparkConfig = new SparkMaxConfig();
            case SparkFlex -> sparkConfig = new SparkFlexConfig();
            default -> throw new IllegalArgumentException("Unknown Spark model!");
        }
        closedLoopConfig = new ClosedLoopConfig();
    }

    public void setInverted(boolean inverted) {
        sparkConfig.inverted(inverted);
        configure(sparkConfig);
    }

    public void follow(SparkBase master) {
        sparkConfig.follow(master);
        configure(sparkConfig);
    }

    public void configPIDF(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings) {
        closedLoopConfig.pidf(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD(),
                feedForwardSettings.getkV());
        sparkConfig.apply(closedLoopConfig);
        configure(sparkConfig);
    }

    public void coast() {
        sparkConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
    }

    public void brake() {
        sparkConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    }
    // @TODO figure out trapezoids

    protected void configure(SparkBaseConfig config) {
        configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}
