package com.spikes2212.dashboard;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.HashSet;
import java.util.List;

/**
 * A class that extends {@link SendableChooser} in order to make an autonomous command chooser.
 *
 * @author Ofri Rosenbaum
 * @see SendableChooser
 */
public class AutoChooser extends SendableChooser<Command> {

    /**
     * The default name that will be given to the {@link Namespace} in case no name was given in the constructor.
     */
    public static final String DEFAULT_NAMESPACE_NAME = "autonomous chooser";

    /**
     * The incrementation value that will be passed when
     * using {@link AutoChooser#AutoChooser(Command, String, Object...)}.
     */
    private static final int INCREMENTATION_VALUE_ONE = 3;

    /**
     * The incrementation value that will be passed when
     * using {@link AutoChooser#AutoChooser(Namespace, Command, String, Object...)}.
     */
    private static final int INCREMENTATION_VALUE_TWO = 4;

    /**
     * The {@link Namespace} that this {@link AutoChooser} will be on.
     */
    protected final Namespace namespace;

    /**
     * A {@link HashSet} that contains the names of all the options in this {@link AutoChooser}.
     */
    private final HashSet<String> names = new HashSet<>();

    /**
     * Creates an {@link AutoChooser} from the given commands, where the first command will be the default option and
     * the rest of the commands will be the rest of the options.
     *
     * <p>All options will have names matching their command's {@link Command#getName()} value. In case of multiple
     * instances of the same class, numbers will also be added to differentiate between the names.</p>
     *
     * @param namespace     the namespace that this {@link AutoChooser} will be on
     * @param defaultOption the command this {@link AutoChooser} will use as the default option
     * @param options       the commands that will be added to this {@link AutoChooser} other than the default command
     */

    public AutoChooser(Namespace namespace, Command defaultOption, Command... options) {
        this.namespace = namespace;
        String defaultName = defaultOption.getName();
        addName(defaultName);
        setDefaultOption(defaultName, defaultOption);
        for (Command option : options) {
            addOption(option.getName(), option);
        }
        putOnShuffleboard();
    }

    /**
     * Creates an {@link AutoChooser} from the given commands, where the first command will be the default option and
     * the rest of the commands will be the rest of the options.
     *
     * <p>All options will have names matching their command's {@link Command#getName()} value. In case of multiple
     * instances of the same class, numbers will also be added to differentiate between the names.</p>
     *
     * @param defaultOption the command this {@link AutoChooser} will use as the default option
     * @param options       the commands that will be added to this {@link AutoChooser} other than the default command
     */
    public AutoChooser(Command defaultOption, Command... options) {
        this(new RootNamespace(DEFAULT_NAMESPACE_NAME), defaultOption, options);
    }


    /**
     * Creates an {@link AutoChooser} where the first {@link Command} and {@link String} will be used as the default
     * option, and the rest of the options after this will be parsed as followed: every even index (0,2,4,...) will be
     * a command and every odd index (1,3,5,...) will be used as the name for the previous command.
     *
     * @param namespace           the namespace that this {@link AutoChooser} will be on
     * @param defaultOption       the command this {@link AutoChooser} will use as the default option
     * @param defaultOptionName   the name for the default command
     * @param incrementationValue the value that should be added to the options' counter to get the correct
     *                            argument index
     * @param options             the rest of the options, where every even index is a command to be added and every odd
     *                            index is the name for the previously mentioned command
     * @throws IllegalArgumentException when the options' length isn't even, an even index is a {@link String}
     *                                  or an odd index is a {@link Command}
     */
    private AutoChooser(Namespace namespace, Command defaultOption, String defaultOptionName,
                        int incrementationValue, Object... options) {
        if (options.length % 2 != 0) throw new IllegalArgumentException("The options' length isn't even.");
        this.namespace = namespace;
        setDefaultOption(defaultOptionName, defaultOption);
        Command command;
        String name;
        int num;
        for (int i = 0; i < options.length; i += 2) {
            if (options[i] instanceof Command) {
                command = (Command) options[i];
            } else {
                num = i + incrementationValue;
                throw new IllegalArgumentException("The " + num + getCorrectSuffix(num) +
                        " argument is not a Command.");
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
     * Creates an {@link AutoChooser} where the first {@link Command} and {@link String} will be used as the default
     * option, and the rest of the options after this will be parsed as followed: every even index (0,2,4,...) will be
     * a command and every odd index (1,3,5,...) will be used as the name for the previous command.
     *
     * @param namespace         the namespace that this {@link AutoChooser} will be on
     * @param defaultOption     the command this {@link AutoChooser} will use as the default option
     * @param defaultOptionName the name for the default command
     * @param options           the rest of the options, where every even index is a command to be added and every odd
     *                          index is the name for the previously mentioned command
     * @throws IllegalArgumentException when the options' length isn't even, an even index is a {@link String}
     *                                  or an odd index is a {@link Command}
     */
    public AutoChooser(Namespace namespace, Command defaultOption, String defaultOptionName, Object... options) {
        this(namespace, defaultOption, defaultOptionName, INCREMENTATION_VALUE_TWO, options);
    }

    /**
     * Creates an {@link AutoChooser} where the first {@link Command} and {@link String} will be used as the default
     * option, and the rest of the options after this will be parsed as followed: every even index (0,2,4,...) will
     * be a command and every odd index (1,3,5,...) will be used as the name for the previous command.
     *
     * @param defaultOption     the command this {@link AutoChooser} will use as the default option
     * @param defaultOptionName the name for the default command
     * @param options           the rest of the options, where every even index is a command to be added and every odd
     *                          index is the name for the previous command
     * @throws IllegalArgumentException when the options' length isn't even, an even index is a {@link String}
     *                                  * or an odd index is a {@link Command}
     */
    public AutoChooser(Command defaultOption, String defaultOptionName, Object... options) {
        this(new RootNamespace(DEFAULT_NAMESPACE_NAME), defaultOption, defaultOptionName, INCREMENTATION_VALUE_ONE,
                options);
    }

    /**
     * Adds the given command with the given name as an option to this {@link AutoChooser}. <br>
     * If this name already exists, a number will be added to differentiate between the options.
     */
    @Override
    public void addOption(String name, Command command) {
        if (!addName(name)) {
            String originalName = name;
            int i = 2;
            name += " " + i;
            while (!addName(name)) {
                i++;
                name = originalName + " " + i;
            }
        }
        super.addOption(name, command);
    }

    /**
     * Schedules the selected command and closes this {@link AutoChooser}.
     */
    public void schedule() {
        getSelected().schedule();
        close();
    }

    /**
     * @param num a positive number
     * @return the correct suffix for the given number
     */
    private String getCorrectSuffix(int num) {
        List<String> suffixes = List.of("th", "st", "nd", "rd", "th", "th", "th", "th", "th", "th");
        switch (num % 100) {
            case 11:
            case 12:
            case 13:
                return "th";
            default:
                return suffixes.get(num % 10);
        }
    }

    private boolean addName(String name) {
        return names.add(name);
    }

    private void putOnShuffleboard() {
        namespace.putData("auto chooser", this);
    }
}
