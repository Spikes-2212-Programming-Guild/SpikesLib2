package com.spikes2212.util.smartmotorcontrollers;

import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public interface SmartMotorController extends MotorController {

    void configurePIDF(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings);

    void configureTrapezoid(TrapezoidProfileSettings trapezoidProfileSettings);

    default void configureLoop(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings,
                               TrapezoidProfileSettings trapezoidProfileSettings) {
        configurePIDF(pidSettings, feedForwardSettings);
        configureTrapezoid(trapezoidProfileSettings);
    }

    void pidSet(UnifiedControlMode controlMode, double setpoint, PIDSettings pidSettings,
                FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings);
}
