package com.spikes2212.utils;

import edu.wpi.first.wpilibj.Sendable;

import java.util.function.Supplier;

public class ChildNamespace extends RootNamespace {

    private Namespace parent;

    public ChildNamespace(String name, Namespace parent) {
        super(name);
        this.parent = parent;
    }

}
