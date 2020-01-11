package com.spikes2212.utils;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.Sendable;

import java.util.function.Supplier;

public class ChildNamespace extends RootNamespace {

    private Namespace parent;

    public ChildNamespace(String name, Namespace parent) {
        super(name);
        this.parent = parent;
    }
    @Override
    public Supplier<Double> addConstantDouble(String name, double value) {
        return parent.addConstantDouble(this.name + " " + name,value);
    }

    @Override
    public Supplier<Integer> addConstantInt(String name, int value) {
        return parent.addConstantInt(this.name + " " + name,value);
    }

    @Override
    public Supplier<String> addConstantString(String name, String value) {
        return parent.addConstantString(this.name + " " + name,value);
    }

    @Override
    public void putData(String key, Sendable value) {
        parent.putData ( name + " " + key,value);
    }

    @Override
    public Sendable getSendable(String key) {
        return parent.getSendable(name+""+key);
    }

    @Override
    public void putString(String key, String value) {
        parent.putString(name+ "" + key,value);
    }

    @Override
    public String getString(String key) {
        return parent.getString(name+ "" + key);
    }

    @Override
    public void putNumber(String key, double value) {
        parent.putNumber(name+ "" + key,value);
    }

    @Override
    public double getNumber(String key) {
        return parent.getNumber(name+ "" +key);
    }

    @Override
    public void putBoolean(String key, boolean value) {
        parent.putBoolean(name+ "" +key,value);
    }

    @Override
    public boolean getBoolean(String key) {
       return parent.getBoolean(name+ "" +key);
    }

}
