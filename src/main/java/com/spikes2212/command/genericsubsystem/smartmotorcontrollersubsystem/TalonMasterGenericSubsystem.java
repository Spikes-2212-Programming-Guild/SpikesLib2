package com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IFollower;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.spikes2212.command.genericsubsystem.GenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

import java.util.List;


public class TalonMasterGenericSubsystem <T extends BaseTalon & MotorController> extends GenericSubsystem
        implements SmartMotorControllerSubsystem {

    protected final T master;
    protected final List<? extends IFollower> slaves;
    protected final PIDSettings pidSettings;
    protected final FeedForwardSettings feedForwardSettings;

    public TalonMasterGenericSubsystem(String namespaceName, double minSpeed, double maxSpeed, T master,
                                       IFollower... slaves) {
        super(namespaceName, minSpeed, maxSpeed);
        this.master = master;
        this.slaves = List.of(slaves);
        this.slaves.forEach(s -> s.follow(master));
        this.pidSettings = new PIDSettings(0, 0, 0);
        this.pidSettings.setIsNull(true);
        this.feedForwardSettings = FeedForwardSettings.EMPTY_FFSETTINGS;
        this.feedForwardSettings.setIsNull(true);
    }

    @Override
    public void configureDashboard() {

    }

    @Override
    protected void apply(double speed) {
        master.set(speed);
    }

    @Override
    public boolean canMove(double speed) {
        return true;
    }

    @Override
    public void stop() {
        master.stopMotor();
    }

    @Override
    public void configureLoop(int slot) {
        checkPIDAndFeedForward(pidSettings, feedForwardSettings);
        master.configFactoryDefault();
        master.config_kP(slot, pidSettings.getkP());
        master.config_kI(slot, pidSettings.getkI());
        master.config_kD(slot, pidSettings.getkD());
        master.config_kF(slot, feedForwardSettings.getkV());
    }

    @Override
    public void pidSet(int slot, ControlMode controlMode, double setpoint) {
        checkPIDAndFeedForward(pidSettings, feedForwardSettings);
        master.configFactoryDefault();
        master.config_kP(slot, pidSettings.getkP());
        master.config_kI(slot, pidSettings.getkI());
        master.config_kD(slot, pidSettings.getkD());
        master.config_kF(slot, feedForwardSettings.getkV());
        master.set(controlMode, setpoint);
    }

    @Override
    public void finish() {

    }

    @Override
    public boolean onTarget(int slot, double setpoint) {
        return false;
    }
}
