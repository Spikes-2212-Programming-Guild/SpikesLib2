package com.spikes2212.command;

import com.spikes2212.dashboard.Namespace;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import java.awt.*;

/**
 * Class that gives more control over LEDs.
 *
 * @author Camellia Lami
 * @see DashboardedSubsystem
 */
public class LEDs extends DashboardedSubsystem {

    /**
     * Creates a {@link AddressableLED} and a {@link AddressableLEDBuffer}.
     */
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    public LEDs(Namespace namespace, AddressableLED led, AddressableLEDBuffer ledBuffer) {
        super(namespace);
        this.led = led;
        this.ledBuffer = ledBuffer;
    }

    public LEDs(String namespaceName, AddressableLED led, AddressableLEDBuffer ledBuffer) {
        this(new RootNamespace(namespaceName), led, ledBuffer);
    }

    /**
     * Sets the LED strip to a specific color
     *
     * @param red   The desired red value.
     * @param green The desired green value.
     * @param blue  The desired blue value.
     */
    public void setStripColor(int red, int green, int blue) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, red, green, blue);
        }
        led.setData(ledBuffer);
    }

    /**
     * @param color The desired {@link Color}.
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
     * Sets a certain range of LEDs to a specific color
     *
     * @param red   The desired red value.
     * @param green The desired green value.
     * @param blue  The desired blue value.
     * @param start The first LED in the range.
     * @param end   The final LED in the range.
     */
    public void setColorInRange(int red, int green, int blue, int start, int end) {
        for (int i = start - 1; i < end; i++) {
            ledBuffer.setRGB(i, red, green, blue);
        }
        led.setData(ledBuffer);
    }

    /**
     * @param color The desired {@link Color}.
     * @param start The first LED in the range.
     * @param end   The final LED in the range.
     */
    public void setColorInRange(Color color, int start, int end) {
        setColorInRange(color.getRed(), color.getGreen(), color.getBlue(), start, end);
    }

    /**
     * Adds any commands or data from this subsystem to the {@link NetworkTable}s.
     */
    @Override
    public void configureDashboard() {
    }
}
