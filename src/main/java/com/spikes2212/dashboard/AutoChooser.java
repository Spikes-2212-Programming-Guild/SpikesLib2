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
     * The default name that will be given to the {@link RootNamespace} in case no name was given in the constructor.
     */
    public static final String DEFAULT_NAMESPACE_NAME = "autonomous chooser";

    /**
     * The {@link RootNamespace} that this {@link SendableChooser} will be on.
     */
    protected RootNamespace rootNamespace;

    /**
     * A {@link HashSet} that contains the names of all the options in this {@link SendableChooser}.
     */
    private final HashSet<String> names = new HashSet<>();

    /**
     * Creates an autonomous chooser from the given commands, where the first command will be the default option and the
     * rest of the commands will be the rest of the options.
     *
     * <p>All options will have names matching their command's {@link Command#getName()}. In case of multiple
     * instances of the same class, numbers will also be added to differentiate between the names.</p>
     *
     * @param rootNamespaceName the name that will be given to the {@link RootNamespace}
     * @param defaultOption the default command this {@link SendableChooser} will use as the default option
     * @param options the commands that will be added to this {@link SendableChooser} other than the default command
     */
    public AutoChooser(String rootNamespaceName, Command defaultOption, Command... options) {
        rootNamespace = new RootNamespace(rootNamespaceName);
        String defaultName = defaultOption.getName();
        addName(defaultName);
        setDefaultOption(defaultName, defaultOption);
        for (Command option : options) {
            addOption(option.getName(), option);
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
        this(DEFAULT_NAMESPACE_NAME, defaultOption, options);
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
        this(rootNamespaceName, defaultOption, defaultOptionName, 4, options);
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
        this(DEFAULT_NAMESPACE_NAME, defaultOption, defaultOptionName, 3, options);
    }

    private AutoChooser(String rootNamespaceName, Command defaultOption, String defaultOptionName,
                        int incrementationValue, Object... options) {
        rootNamespace = new RootNamespace(rootNamespaceName);
        setDefaultOption(defaultOptionName, defaultOption);
        Command command;
        String name;
        int num;
        for (int i = 0; i < options.length - options.length % 2; i += 2) {
            if (options[i] instanceof Command) {
                command = (Command) options[i];
            } else {
                num = i + incrementationValue;
                throw new IllegalArgumentException("The " + num + getCorrectSuffix(num) + " argument is not a Command.");
            }
            if (options[i + 1] instanceof String) {
                name = (String) options[i + 1];
            } else {
                num = i + incrementationValue + 1;
                throw new IllegalArgumentException("The " + num + getCorrectSuffix(num) + " argument is not a String.");
            }
            addOption(name, command);
        }
        putOnShuffleboard();
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
                i++;
                name = name.substring(0, name.length() - String.valueOf(i-1).length()) + i;
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

    private String getCorrectSuffix(int num) {
        switch (num % 10) {
            case 1: {
                if (num == 11) return "th";
                return "st";
            }
            case 2: {
                if (num == 12) return "th";
                return "nd";
            }
            case 3: {
                if (num == 13) return "th";
                return "rd";
            }
            default:
                return "th";
        }
    }

    private boolean addName(String name) {
        return names.add(name);
    }

    private void putOnShuffleboard() {
        rootNamespace.putData("auto chooser", this);
    }
}
