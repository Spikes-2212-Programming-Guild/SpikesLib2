package com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem;

import com.spikes2212.command.genericsubsystem.GenericSubsystem;

public class SparkMaxGenericSubsystem extends GenericSubsystem implements SmartMotorControllerSubsystem {

    public SparkMaxGenericSubsystem(String namespaceName) {
        super(namespaceName);
    }

    @Override
    public void configureDashboard() throws UnassignedFeedForwardSettingsException, UnassignedPIDSettingsException {

    }

    @Override
    protected void apply(double speed) {

    }

    @Override
    public boolean canMove(double speed) {
        return false;
    }

    @Override
    public void stop() {

    }
}
