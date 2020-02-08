package com.spikes2212.dashboard;

import edu.wpi.first.wpilibj.Sendable;

import java.util.function.Supplier;

public class ChildNamespace extends RootNamespace {

    private Namespace parent;
    private String separator;

    public ChildNamespace(String name, Namespace parent) {
        super(name);
        this.parent = parent;
        separator = "/";
    }

    public ChildNamespace(String name, Namespace parent, String separator) {
        super(name);
        this.parent = parent;
        this.separator = separator;
    }

    @Override
    public Supplier<Double> addConstantDouble(String name, double value) {
        return parent.addConstantDouble(this.name + separator + name, value);
    }

    @Override
    public Supplier<Integer> addConstantInt(String name, int value) {
        return parent.addConstantInt(this.name + separator + name, value);
    }

    @Override
    public Supplier<String> addConstantString(String name, String value) {
        return parent.addConstantString(this.name + separator + name, value);
    }

    @Override
    public void putData(String key, Sendable value) {
        parent.putData(name + separator + key, value);
    }

    @Override
    public Sendable getSendable(String key) {
        return parent.getSendable(name + separator + key);
    }

    @Override
    public void putString(String key, Supplier<String> value) {
        parent.putString(name + separator + key, value);
    }

    @Override
    public String getString(String key) {
        return parent.getString(name + separator + key);
    }

    @Override
    public void putNumber(String key, Supplier<? extends Number> value) {
        parent.putNumber(name + separator + key, value);
    }

    @Override
    public double getNumber(String key) {
        return parent.getNumber(name + separator + key);
    }

    @Override
    public void putBoolean(String key, Supplier<Boolean> value) {
        parent.putBoolean(name + separator + key, value);
    }

    @Override
    public boolean getBoolean(String key) {
        return parent.getBoolean(name + separator + key);
    }

}
