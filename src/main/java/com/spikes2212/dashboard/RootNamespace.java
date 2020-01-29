package com.spikes2212.dashboard;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

public class RootNamespace implements Namespace {
    private final Map<String, Sendable> TABLES_TO_DATA = new HashMap<>();

    protected String name;
    private Map<String, Supplier<String>> stringFields;
    private Map<String, Supplier<? extends Number>> numberFields;
    private Map<String, Supplier<Boolean>> booleanFields;
    private NetworkTable table;

    public RootNamespace(String name) {
        this.name = name;
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        this.table = inst.getTable(this.name);
        stringFields = new HashMap<>();
        numberFields = new HashMap<>();
        booleanFields = new HashMap<>();
    }

    @Override
    public Supplier<Double> addConstantDouble(String name, double value) {
        NetworkTableEntry entry = table.getEntry(name);
        if (! table.containsKey(name)) {
            entry.setDouble(value);
            entry.setPersistent();
        }
        return () -> entry.getDouble(value);
    }

    @Override
    public Supplier<Integer> addConstantInt(String name, int value) {
        NetworkTableEntry entry = table.getEntry(name);
        if (! table.containsKey(name)) {
            entry.setNumber(value);
            entry.setPersistent();
        }
        return () -> entry.getNumber(value).intValue();
    }

    @Override
    public Supplier<String> addConstantString(String name, String value) {
        NetworkTableEntry entry = table.getEntry(name);
        if (! table.containsKey(name)) {
            entry.setString(value);
            entry.setPersistent();
        }
        return () -> entry.getString(value);
    }

    @Override
    public Namespace addChild(String name) {
        return new ChildNamespace(name, this);
    }

    @Override
    public void putData(String key, Sendable value) {
        Sendable sddata = TABLES_TO_DATA.get(key);
        if (sddata == null || sddata != value) {
            TABLES_TO_DATA.put(key, value);
            NetworkTable dataTable = table.getSubTable(key);
            SendableRegistry.publish(value, dataTable);
            dataTable.getEntry(".name").setString(key);
        }
    }

    @Override
    public Sendable getSendable(String key) {
        NetworkTableEntry entry = this.table.getEntry(key);
        NetworkTableValue value = entry.getValue();
        return (Sendable) value.getValue();
    }

    @Override
    public void putString(String key, Supplier<String> value) {
        remove(key);
        NetworkTableEntry entry = this.table.getEntry(key);
        entry.setString(value.get());
        stringFields.put(key, value);
    }

    @Override
    public String getString(String key) {
        NetworkTableEntry entry = this.table.getEntry(key);
        NetworkTableValue value = entry.getValue();
        return value.getString();
    }

    @Override
    public void putNumber(String key, Supplier<? extends Number> value) {
        remove(key);
        NetworkTableEntry entry = this.table.getEntry(key);
        entry.setNumber(value.get());
        numberFields.put(key, value);
    }

    @Override
    public double getNumber(String key) {
        NetworkTableEntry entry = this.table.getEntry(key);
        NetworkTableValue value = entry.getValue();
        return value.getDouble();
    }

    @Override
    public void putBoolean(String key, Supplier<Boolean> value) {
        remove(key);
        NetworkTableEntry entry = this.table.getEntry(key);
        entry.setBoolean(value.get());
        booleanFields.put(key, value);
    }

    @Override
    public boolean getBoolean(String key) {
        NetworkTableEntry entry = this.table.getEntry(key);
        NetworkTableValue value = entry.getValue();
        return value.getBoolean();
    }

    public void remove(String name) {
        stringFields.remove(name);
        numberFields.remove(name);
        booleanFields.remove(name);
    }

    private void updateString() {
        for (Map.Entry<String, Supplier<String>> map : stringFields.entrySet()) {
            NetworkTableEntry entry = this.table.getEntry(map.getKey());
            entry.setString(map.getValue().get());
        }
    }

    private void updateNumber() {
        for (Map.Entry<String, Supplier<? extends Number>> map : numberFields.entrySet()) {
            NetworkTableEntry entry = this.table.getEntry(map.getKey());
            entry.setNumber(map.getValue().get());
        }
    }

    private void updateBoolean() {
        for (Map.Entry<String, Supplier<Boolean>> map : booleanFields.entrySet()) {
            NetworkTableEntry entry = this.table.getEntry(map.getKey());
            entry.setBoolean(map.getValue().get());
        }
    }

    public void update() {
        updateBoolean();
        updateNumber();
        updateString();
    }
}
