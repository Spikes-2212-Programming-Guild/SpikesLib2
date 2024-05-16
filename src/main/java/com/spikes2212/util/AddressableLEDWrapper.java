package com.spikes2212.util;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import java.awt.Color;

/**
 * A class that handles the communication between a LED strip and the code.
 * This class still requires further testing and an update will be made as soon as testing is finished.
 *
 * @author Camellia Lami
 */
public class AddressableLEDWrapper {

    /**
     * The controlled LED strip.
     */
    private final AddressableLED led;

    /**
     * The controlled LED strip's data.
     */
    private final AddressableLEDBuffer ledBuffer;

    /**
     * Whether the LED strip should update its data by itself.
     */
    private boolean updateAutomatically;

    public AddressableLEDWrapper(int ledPort, int numberOfLEDs, boolean updateAutomatically) {
        this.led = new AddressableLED(ledPort);
        this.ledBuffer = new AddressableLEDBuffer(numberOfLEDs);
        this.updateAutomatically = updateAutomatically;
    }

    public AddressableLEDWrapper(int ledPort, int numberOfLEDs) {
        this(ledPort, numberOfLEDs, true);
    }

    public void periodic() {
        if (updateAutomatically) update();
    }

    public void updateAutomatically(boolean shouldUpdate) {
        updateAutomatically = shouldUpdate;
    }

    /**
     * Sets the LED strip to a specific color.
     *
     * @param red   the red value from 0 to 255
     * @param green the green value from 0 to 255
     * @param blue  the blue value from 0 to 255
     */
    public void setStripColor(int red, int green, int blue) {
        setColorInRange(0, ledBuffer.getLength() - 1, red, green, blue);
    }

    /**
     * Sets the LED strip to a specific color.
     *
     * @param color the desired {@link Color}
     */
    public void setStripColor(Color color) {
        setStripColor(color.getRed(), color.getGreen(), color.getBlue());
    }

    /**
     * Turns off the strip.
     */
    public void turnOff() {
        setStripColor(0, 0, 0);
    }

    /**
     * Sets a certain range of LEDs to a specific color.
     *
     * @param start the first LED in the range
     * @param end   the final LED in the range
     * @param red   the red value from 0 to 255
     * @param green the green value from 0 to 255
     * @param blue  the blue value from 0 to 255
     */
    public void setColorInRange(int start, int end, int red, int green, int blue) {
        for (int i = start; i <= end; i++) {
            ledBuffer.setRGB(i, red, green, blue);
        }
    }

    /**
     * Sets a certain range of LEDs to a specific color.
     *
     * @param start the first LED in the range
     * @param end   the final LED in the range
     * @param color the desired {@link Color}
     */
    public void setColorInRange(int start, int end, Color color) {
        setColorInRange(start, end, color.getRed(), color.getGreen(), color.getBlue());
    }

    /**
     * Sets a specific LED to a specific color.
     *
     * @param index the index of the LED
     * @param red   the red value from 0 to 255
     * @param green the green value from 0 to 255
     * @param blue  the blue value from 0 to 255
     */
    public void setColorAt(int index, int red, int green, int blue) {
        ledBuffer.setRGB(index, red, green, blue);
    }

    /**
     * Sets a specific LED to a specific color.
     *
     * @param index the index of the LED
     * @param color the desired {@link Color}
     */
    public void setColorAt(int index, Color color) {
        ledBuffer.setRGB(index, color.getRed(), color.getGreen(), color.getBlue());
    }

    /**
     * @param index the index of the LED
     * @return the {@link Color} of said LED
     */
    public Color getColorAt(int index) {
        return new Color((int) ledBuffer.getLED(index).red * 255,
                (int) ledBuffer.getLED(index).green * 255,
                (int) ledBuffer.getLED(index).blue * 255);
    }

    /**
     * @param index
     * @return the red value of said LED
     */
    public int getRedAt(int index) {
        return getColorAt(index).getRed();
    }

    /**
     * @param index
     * @return the green value of said LED
     */
    public int getGreenAt(int index) {
        return getColorAt(index).getGreen();
    }

    /**
     * @param index
     * @return the blue value of said LED
     */
    public int getBlueAt(int index) {
        return getColorAt(index).getBlue();
    }

    /**
     * Takes the buffer's data and applies it to the LED strip.
     * This method should be called periodically.
     */
    public void update() {
        led.setData(ledBuffer);
    }

    /**
     * Activates the LED strip.
     */
    public void enableStrip() {
        led.start();
    }

    /**
     * Deactivates the LED strip.
     */
    public void disableStrip() {
        led.stop();
    }

    /**
     * Advances all the LED lights on the strip.
     *
     * @param advanceBy how many lights should the LED advance by
     * @param loop      whether LED lights should return to the start after leaving the strip
     */
    public void advance(int advanceBy, boolean loop) {
        AddressableLEDBuffer newBuffer = new AddressableLEDBuffer(ledBuffer.getLength());
        if (loop) {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                newBuffer.setLED((i + advanceBy) % ledBuffer.getLength(), ledBuffer.getLED(i));
            }
        } else {
            for (int i = advanceBy; i < ledBuffer.getLength(); i++) {
                newBuffer.setLED(i, ledBuffer.getLED(i));
            }
        }
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED(i, newBuffer.getLED(i));
        }
    }

    public void invert() {
        AddressableLEDBuffer newBuffer = new AddressableLEDBuffer(ledBuffer.getLength());
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            newBuffer.setLED(i, ledBuffer.getLED(ledBuffer.getLength() - i - 1));
        }
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED(i, newBuffer.getLED(i));
        }
    }
}
