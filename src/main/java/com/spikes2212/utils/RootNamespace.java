package com.spikes2212.utils;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Sendable;

import java.util.function.Supplier;

public class RootNamespace implements Namespace {

    protected String name;
    private NetworkTable table;

    public RootNamespace(String name) {
        this.name = name;
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        this.table = inst.getTable(this.name);
    }

    @Override
    public Supplier<Double> addConstantDouble(String name, double value) {
        NetworkTableEntry entry =this.table.getEntry(name);
        entry.setDouble(value);
        return () -> value;
    }

    @Override
    public Supplier<Integer> addConstantInt(String name, int value) {
        NetworkTableEntry entry =this.table.getEntry(name);
        entry.setNumber(value);
        return () -> value;
    }

    @Override
    public Supplier<String> addConstantString(String name, String value) {
        NetworkTableEntry entry =this.table.getEntry(name);
        entry.setString(value);
        return () ->value;
    }

    @Override
    public Namespace addChild(String name) {

        return new ChildNamespace(name, this);
    }

    @Override
    public void putData(String key, Sendable value) {
        NetworkTableEntry entry =this.table.getEntry(key);
        entry.setValue(value);
    }

    @Override
    public Sendable getSendable(String key) {
        NetworkTableEntry entry =this.table.getEntry(key);
        NetworkTableValue value =entry.getValue();
        return (Sendable) value.getValue();
    }

    @Override
    public void putString(String key, String value) {
        NetworkTableEntry entry =this.table.getEntry(key);
        entry.setString(value);
    }

    @Override
    public String getString(String key) {
        NetworkTableEntry entry =this.table.getEntry(key);
        NetworkTableValue value=entry.getValue();
        return value.getString();
    }

    @Override
    public void putNumber(String key, double value) {
        NetworkTableEntry entry =this.table.getEntry(key);
        entry.setDouble(value);
    }

    @Override
    public double getNumber(String key) {
        NetworkTableEntry entry =this.table.getEntry(key);
        NetworkTableValue value =entry.getValue();
        return value.getDouble();
    }

    @Override
    public void putBoolean(String key, boolean value) {
        NetworkTableEntry entry =this.table.getEntry(key);
        entry.setBoolean(value);
    }

    @Override
    public boolean getBoolean(String key) {
        NetworkTableEntry entry =this.table.getEntry(key);
        NetworkTableValue value = entry.getValue();
        return value.getBoolean();
    }
}
