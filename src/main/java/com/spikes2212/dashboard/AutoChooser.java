package com.spikes2212.dashboard;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.HashSet;

/**
 * A class that implements {@link SendableChooser} in order to make an autonomous command chooser.
 *
 * @author Ofri Rosenbaum
 * @see SendableChooser
 */
public class AutoChooser extends SendableChooser<Command> {

    /**
     * A {@link HashSet} that contains the names of all the options in this {@link SendableChooser}.
     */
    private HashSet<String> names = new HashSet<>();

    /**
     * The {@link RootNamespace} that this {@link SendableChooser} will be on.
     */
    private RootNamespace rootNamespace;

    /**
     * Creates an autonomous chooser from the given commands, where the first command will be the default option and the
     * rest of the commands will be the rest of the options.
     *
     * <p>All options will have names matching their command's {@link Class#getSimpleName()}. In case of multiple
     * instances of the same class, numbers will also be added to differentiate between the names.</p>
     *
     * @param rootNamespaceName the name that will be given to the {@link RootNamespace}
     * @param defaultOption the default command this {@link SendableChooser} will use as the default option
     * @param options the commands that will be added to this {@link SendableChooser} other than the default command
     */
    public AutoChooser(String rootNamespaceName, Command defaultOption, Command... options) {
        rootNamespace = new RootNamespace(rootNamespaceName);
        String defaultName = defaultOption.getClass().getSimpleName();
        addName(defaultName);
        setDefaultOption(defaultName, defaultOption);
        for (Command option : options) {
            addOption(option.getClass().getSimpleName(), option);
        }
        putOnShuffleboard();
    }

    /**
     * Creates an autonomous chooser from the given commands, where the first command will be the default option and the
     * rest of the commands will be the rest of the options.
     *
     * <p>All options will have names matching their command's {@link Class#getSimpleName()}. In case of multiple
     * instances of the same class, numbers will also be added to differentiate between the names.</p>
     *
     * @param defaultOption the default command this {@link SendableChooser} will use as the default option
     * @param options the commands that will be added to this {@link SendableChooser} other than the default command
     */
    public AutoChooser(Command defaultOption, Command... options) {
        this("autonomous chooser", defaultOption, options);
    }

    /**
     * Creates an autonomous chooser where the first {@link Command} and {@link String} will be used as the default
     * option, and the rest of the options will be parsed as following: every even index (0,2,4,...) will be a command
     * and every odd index (1,3,5,...) will be used as the name for the previous command.
     *
     * @param rootNamespaceName the name that will be given to the {@link RootNamespace}
     * @param defaultOption the default command this {@link SendableChooser} will use as the default option
     * @param defaultOptionName the name for the default command
     * @param options the rest of the options, where every even index is a command to be added and every odd index is
     *               the name for the previously mentioned command
     */
    public AutoChooser(String rootNamespaceName, Command defaultOption, String defaultOptionName, Object... options) {
        rootNamespace = new RootNamespace(rootNamespaceName);
        setDefaultOption(defaultOptionName, defaultOption);
        Command command;
        String name;
        for (int i = 0; i < options.length - options.length % 2; i+=2) {
            if (options[i] instanceof Command) {
                command = (Command) options[i];
            } else {
                throw new IllegalArgumentException("The " + (i + 3) + " argument is not a Command.");
            }
            if (options[i + 1] instanceof String) {
                name = (String) options[i + 1];
            } else {
                throw new IllegalArgumentException("The " + (i + 4) + " argument is not a String.");
            }
            addOption(name, command);
        }
        putOnShuffleboard();
    }

    /**
     * Creates an autonomous chooser where the first {@link Command} and {@link String} will be used as the default
     * option, and the rest of the options will be parsed as following: every even index (0,2,4,...) will be a command
     * and every odd index (1,3,5,...) will be used as the name for the previous command.
     *
     * @param defaultOption the default command this {@link SendableChooser} will use as the default option
     * @param defaultOptionName the name for the default command
     * @param options the rest of the options, where every even index is a command to be added and every odd index is
     *               the name for the previous command
     */
    public AutoChooser(Command defaultOption, String defaultOptionName, Object... options) {
        this("autonomous chooser", defaultOption, defaultOptionName, options);
    }

    /**
     * Adds the given command with the given name as an option to this autonomous chooser. <br>
     * In case this name already exists, a number will be added to differentiate between the options.
     */
    @Override
    public void addOption(String name, Command command) {
        if (!addName(name)) {
            int i = 2;
            name += " " + i;
            while (!addName(name)) {
                name = name.replace(String.valueOf(i), String.valueOf(i++));
            }
        }
        super.addOption(name, command);
    }

    /**
     * Schedules the selected command and closes this {@link SendableChooser}.
     */
    public void schedule() {
        getSelected().schedule();
        close();
    }

    private boolean addName(String name) {
        return names.add(name);
    }

    private void putOnShuffleboard() {
        rootNamespace.putData("auto chooser", this);
    }
}
