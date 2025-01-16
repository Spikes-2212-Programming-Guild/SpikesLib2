package com.spikes2212.util;

import com.revrobotics.spark.SparkLowLevel;

public class SparkMaxWrapper extends SparkBaseWrapper {
    public SparkMaxWrapper(int deviceId, MotorType type) {
        super(deviceId, type, SparkModel.SparkMax);
    }
}
