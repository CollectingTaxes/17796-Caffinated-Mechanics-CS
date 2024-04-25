package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Wheel extends Subsystem {
    public static class Constants {




        public static Hardware hardware;
        private static class Hardware {

            public static Servo.Direction DIRECTION = Servo.Direction.FORWARD;

        }

        public static Position position;
        private static class Position {
            public static double ON = 1;
            public static double OFF = 0;
        }


    }

    private final CRServo wheel;

    public Wheel(@NonNull OpMode opMode) {
        super(opMode);
        wheel = opMode.hardwareMap.get(CRServo.class, "gripper");
        wheel.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    protected void manualControl() {
        if (opMode.gamepad1.right_trigger > 0.2) on();
        else if (opMode.gamepad1.right_trigger < 0.2) off();
    }

    public void on() {
        wheel.setPower(Constants.Position.ON);
    }

    public void off() {
        wheel.setPower(Constants.Position.OFF);
    }
}
