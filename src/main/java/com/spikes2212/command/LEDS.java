package com.spikes2212.command;

import com.spikes2212.dashboard.Namespace;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * Class that gives more control over LEDS.
 *
 * @author Camellia Lami
 * @see DashboardedSubsystem
 */
public class LEDS extends DashboardedSubsystem {

    /**
     * Creates a {@link AddressableLED} and a {@link AddressableLEDBuffer}.
     */
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    public LEDS(Namespace namespace, AddressableLED led, AddressableLEDBuffer ledBuffer) {
        super(namespace);
        this.led = led;
        this.ledBuffer = ledBuffer;
    }

    public LEDS(String namespaceName, AddressableLED led, AddressableLEDBuffer ledBuffer) {
        this(new RootNamespace(namespaceName), led, ledBuffer);
    }

    /**
     * Sets the led strip to a specific color
     *
     * @param red   The desired red value.
     * @param green The desired green value.
     * @param blue  The desired blue value.
     */
    public void setStripColor(int red, int green, int blue) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, red, green, blue);
        }
    }

    /**
     * Turns off the strip.
     */
    public void turnOff() {
        setStripColor(0, 0, 0);
    }

    /**
     * Adds any commands or data from this subsystem to the {@link NetworkTable}s.
     */
    @Override
    public void configureDashboard() {
    }
}
