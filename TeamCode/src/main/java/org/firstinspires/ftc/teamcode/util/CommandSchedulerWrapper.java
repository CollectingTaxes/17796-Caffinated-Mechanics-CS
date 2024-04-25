package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Button;

import java.util.function.BooleanSupplier;

public class CommandSchedulerWrapper {
    public CommandSchedulerWrapper() {
    }

    public Button add(BooleanSupplier button) {
        return new BooleanButton(button);
    }

    public void addDefault(Runnable toRun) {
        CommandScheduler.getInstance().addButton(toRun);
    }
}