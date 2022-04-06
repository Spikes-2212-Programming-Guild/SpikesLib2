package com.spikes2212.dashboard;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.HashSet;

/**
 * A class that implements the {@link #SendableChooser} class in order to make an autonomous command chooser.
 *
 * @author Ofri Rosenbaum
 * @see SendableChooser
 */
public class AutoChooser extends SendableChooser<Command> {

    public AutoChooser(Command defaultOption, Command... options) {
        HashSet<String> names = new HashSet<>();
        String defaultName = defaultOption.getClass().getSimpleName();
        names.add(defaultName);
        setDefaultOption(defaultName, defaultOption);
        for (Command option : options) {
            String name = option.getClass().getSimpleName();
            if (names.add(name)) {
                addOption(name, option);
            } else {
                int i = 2;
                name += " " + i;
                while (!names.add(name)) {
                    name = name.replace(String.valueOf(i), String.valueOf(i++));
                }
                addOption(name, option);
            }
        }
    }
}
