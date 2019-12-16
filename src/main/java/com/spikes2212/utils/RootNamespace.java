package com.spikes2212.utils;

import edu.wpi.first.wpilibj.Sendable;

import java.util.function.Supplier;

public class RootNamespace implements Namespace {

    private String name;

    public RootNamespace(String name) {
        this.name = name;
    }

    @Override
    public Supplier<Double> addConstantDouble(String name, double value) {
        return null;
    }

    @Override
    public Supplier<Integer> addConstantInt(String name, int value) {
        return null;
    }

    @Override
    public Supplier<String> addConstantString(String name, String value) {
        return null;
    }

    @Override
    public Namespace addChild(String name) {
        return new ChildNamespace(this.name + '/' + name);
    }

    @Override
    public void putData(String key, Sendable value) {

    }

    @Override
    public Sendable getSendable(String key) {
        return null;
    }

    @Override
    public void putString(String key, String value) {

    }

    @Override
    public String getString(String key) {
        return null;
    }

    @Override
    public void putNumber(String key, double value) {

    }

    @Override
    public double getNumber(String key) {
        return 0;
    }

    @Override
    public void putBoolean(String key, boolean value) {

    }

    @Override
    public boolean getBoolean(String key) {
        return false;
    }
}
