package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.subsystem.Wheel.WheelConstants.*;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Wheel extends HardwareSubsystem {

    private final CRServo Wheel;

    public static class WheelConstants {
        public static Hardware hardware = new Hardware();
        public static Value value = new Value();

        public static class Hardware {
            public String ID = "Wheel";
            public boolean REVERSED = true;
        }
        public static class Value {
            public double INTAKE                       = 1; // Degrees
            public double OUTTAKE                      = -1; // Degrees
        }
    }

    public Wheel(OpMode opMode) {
        super(opMode);
        Wheel = new CRServo(hardwareMap, hardware.ID);
        Wheel.setInverted(hardware.REVERSED);
    }

    public void open() {
        Wheel.set(value.INTAKE);
    }
    public void close() {
        Wheel.set(value.OUTTAKE);
    }

    public void off(){Wheel.set(0);}

    @Override
    public void init() {

    }

    @Override
    public void periodic() {

    }
}