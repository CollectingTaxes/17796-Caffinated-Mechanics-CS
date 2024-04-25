package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.button.Button;

import java.util.function.BooleanSupplier;

public class BooleanButton extends Button {

    private final BooleanSupplier booleanSupplier;

    public BooleanButton(@NonNull BooleanSupplier booleanSupplier) {
        this.booleanSupplier = booleanSupplier;
    }
    @Override
    public boolean get() {
        return booleanSupplier.getAsBoolean();
    }
}