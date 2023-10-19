package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.tools.javac.jvm.Gen;

public class Slides {
    private final DcMotor leftSlide, rightSlide;
    private final Gamepad gamepad2;
    private final int Gear_Ratio = 1;
    private final int Gear_Diameter_Centimeters = 1;


    public Slides(HardwareMap hardwareMap, Gamepad gamepad2) {
        leftSlide = (DcMotor) hardwareMap.get("leftSlide");
        rightSlide = (DcMotor) hardwareMap.get("rightSlide");
        this.gamepad2 = gamepad2;

        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        //TODO:Test for the other mode Float
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setTargetPosition(0);
        rightSlide.setTargetPosition(0);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(1);
        rightSlide.setPower(1);
    }
    public void teleOp() {
            //Depends on number of ticks per revolution
        if (gamepad2.a) slideMotors(((1000 * Gear_Ratio) / (Gear_Diameter_Centimeters * Math.PI)));
        else if (gamepad2.b)slideMotors(500);
        else if (gamepad2.y) slideMotors(250);
        else if (gamepad2.x) slideMotors(0);
    }
    public void slideMotors(double targetPosition) {
        //Centimeters = (GEAR_RATIO * TICKS_PER_REVOLUTION) / (GEAR_DIAMETER_CENTIMETERS * Math.PI);
        leftSlide.setTargetPosition((int) targetPosition);
        rightSlide.setTargetPosition((int) targetPosition);
    }
}