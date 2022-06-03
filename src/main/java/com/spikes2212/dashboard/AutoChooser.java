package com.spikes2212.dashboard;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.HashSet;

/**
 * A class that implements the {@link SendableChooser} class in order to make an autonomous command chooser.
 *
 * @author Ofri Rosenbaum
 * @see SendableChooser
 */
public class AutoChooser extends SendableChooser<Command> {

    /**
     * Creates an autonomous chooser from the given commands, where the first command will be the default and the rest
     * of the commands will be the rest of the options.
     *
     * <p>All options will have names matching their command's {@link Command#getName}. In case of multiple instances
     * of the same class, numbers will also be added to differentiate between the names.</p>
     *
     * @param defaultOption the default command this {@link SendableChooser} will use as the default option
     * @param options the commands that will be added to this {@link SendableChooser} other than the default command
     */
    public AutoChooser(Command defaultOption, Command... options) {
        HashSet<String> names = new HashSet<>();
        String defaultName = defaultOption.getClass().getSimpleName();
        names.add(defaultName);
        setDefaultOption(defaultName, defaultOption);
        for (Command option : options) {
            String name = option.getName();
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

    public AutoChooser(Command defaultOption, String defaultOptionName, Object... options) {
        setDefaultOption(defaultOptionName, defaultOption);
        for (int i = 0; i < options.length - options.length % 2; i++) {
            Command command;
            String name;
            if (options[i] instanceof Command) {
                command = (Command) options[i];
            } else {
                throw new IllegalArgumentException("The " + (i * 2 + 1) + " argument is not a command.");
            }
            if (options[i + 1] instanceof String) {
                name = (String) options[i + 1];
            } else {
                throw new IllegalArgumentException("The " + (i * 2 + 2) + " argument is not a string.");
            }
            addOption(name, command);
        }
    }

    public void schedule() {
        getSelected().schedule();
        close();
    }
}
