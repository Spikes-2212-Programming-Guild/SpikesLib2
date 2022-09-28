package com.spikes2212.util;

/**
 * D-PAD values.
 *
 * @author Ofri Rosenbaum
 */
public enum DPAD {

    UP(0),
    UPPER_RIGHT(45),
    RIGHT(90),
    LOWER_RIGHT(135),
    DOWN(180),
    LOWER_LEFT(225),
    LEFT(270),
    UPPER_LEFT(315);

    public final int VALUE;

    DPAD(int value) {
        this.VALUE = value;
    }
}
