package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Slides extends HardwareSubsystem {
    DcMotorEx leftLiftMotor;
    DcMotorEx rightLiftMotor;
    ServoEx leftLiftServo;
    ServoEx rightLiftServo;
    double motorPosition;
    double servoPosition;

    public static class LiftMotorConstants {
        public static Hardware hardware = new Hardware();
        public static Controller controller = new Controller();
        public static Position position = new Position();
        public static Speed speed = new Speed();

        public static class Hardware {
            public String LEFT_ID = "leftLift";
            public String RIGHT_ID = "rightLift";
            public boolean LEFT_REVERSED = true;
            public boolean RIGHT_REVERSED = false;
            public double
                    RPM           = 312,
                    CPR           = 547.7;
        }

        public static class Controller {
            public double
                    TOLERANCE     = 8,
                    POSITION_TOLERANCE = 8,
                    KP            = 4,
                    kI            = 0,
                    kD            = 0,
                    kF            = 0;
        }
        public static class Position {
            public double
                    TALL = 100, // Degrees
                    MIDDLE = 100, // Degrees
                    LOWER = 100, // Degrees
                    INITIAL = 0,
                    MAX_POSITION = 100,
                    MIN_POSITION = 0;
        }
        public static class Speed {
            public double
                    NORMAL_SPEED         = 1,
                    SPEED_DEGREES_CHANGE          = 5;

        }
    }

    public static class LiftServoConstants {
        public static Hardware hardware = new Hardware();
        public static Position position = new Position();
        public static Speed speed = new Speed();

        public static class Hardware {
            public String LEFT_ID            = "leftServo";
            public String RIGHT_ID            = "rightServo";
            public boolean LEFT_REVERSED     = false;
            public boolean RIGHT_REVERSED     = true;
        }

        public static class Position {
            public double
                    HIGH          = 50, // Degrees
                    MID           = 50, // Degrees
                    LOW           = 50, // Degrees
                    INITIAL = 6,
                    RIGHT_SERVO_OFFSET = 0;
        }
        public static class Speed {
            public double
                    SPEED_DEGREES_CHANGE          = 0.4;

        }
    }

    public Slides(OpMode opMode) {
        super(opMode);
        leftLiftMotor = hardwareMap.get(DcMotorEx.class, LiftMotorConstants.hardware.LEFT_ID);
        leftLiftMotor.setDirection(LiftMotorConstants.hardware.LEFT_REVERSED ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftLiftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(10, 0, 0, 0));
//        leftLiftMotor.setTargetPositionTolerance();



        rightLiftMotor = hardwareMap.get(DcMotorEx.class, LiftMotorConstants.hardware.RIGHT_ID);
        rightLiftMotor.setDirection(LiftMotorConstants.hardware.RIGHT_REVERSED ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftLiftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(10, 0, 0, 0));
//        leftLiftMotor.setTargetPositionTolerance();

        leftLiftServo = new SimpleServo(hardwareMap, LiftServoConstants.hardware.LEFT_ID, 0, 270);
        leftLiftServo.setInverted(LiftServoConstants.hardware.LEFT_REVERSED);

        rightLiftServo = new SimpleServo(hardwareMap, LiftServoConstants.hardware.RIGHT_ID, 0, 270);
        rightLiftServo.setInverted(LiftServoConstants.hardware.RIGHT_REVERSED);
        initial();
    }


    public void high() {
        motorPosition = LiftMotorConstants.position.TALL;
        servoPosition = LiftServoConstants.position.HIGH;
        turnToPositions();
    }

    public void mid() {
        motorPosition = LiftMotorConstants.position.MIDDLE;
        servoPosition = LiftServoConstants.position.MID;
        turnToPositions();
    }

    public void low() {
        motorPosition = LiftMotorConstants.position.LOWER;
        servoPosition = LiftServoConstants.position.LOW;
        turnToPositions();
    }

    public void initial() {
        motorPosition = LiftMotorConstants.position.INITIAL;
        servoPosition = LiftServoConstants.position.INITIAL;
        turnToPositions();
    }









    public void setMotorAngle(double degrees) {
        leftLiftMotor.setTargetPosition((int) (LiftMotorConstants.hardware.CPR / 360 * degrees));
        rightLiftMotor.setTargetPosition((int) (LiftMotorConstants.hardware.CPR / 360 * degrees));
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void increaseMotorPosition () {
        changeMotorPosition(LiftMotorConstants.speed.SPEED_DEGREES_CHANGE);
        turnToPositions();
    }

    public void decreaseMotorPosition () {
        changeMotorPosition(-LiftMotorConstants.speed.SPEED_DEGREES_CHANGE);
        turnToPositions();
    }

    public void changeMotorPosition(double degrees) {
        if (motorPosition >= LiftMotorConstants.position.MAX_POSITION) {
            if (degrees < 0) motorPosition += degrees;
            return;
        }

        if (motorPosition <= LiftMotorConstants.position.MIN_POSITION) {
            if (degrees > 0) motorPosition += degrees;
            return;
        }

        motorPosition += degrees;
    }



    public void increaseServoPosition () {
        changeServoPosition(LiftServoConstants.speed.SPEED_DEGREES_CHANGE);
        turnToPositions();
    }

    public void decreaseServoPosition () {
        changeServoPosition(-LiftServoConstants.speed.SPEED_DEGREES_CHANGE);
        turnToPositions();
    }

    public void changeServoPosition(double degrees) {
        servoPosition += degrees;
    }

    public void turnToPositions() {
        leftLiftServo.turnToAngle(servoPosition);
        rightLiftServo.turnToAngle(servoPosition + LiftServoConstants.position.RIGHT_SERVO_OFFSET);
        setMotorAngle(motorPosition);

        leftLiftMotor.setPower(LiftMotorConstants.speed.NORMAL_SPEED);
        rightLiftMotor.setPower(LiftMotorConstants.speed.NORMAL_SPEED);
    }

    @Override
    public void init() {

    }

    @Override
    protected void manualControl() {
        if (opMode.gamepad2.dpad_up) {
            high();
        }
        else if (opMode.gamepad2.dpad_right) {
            mid();
        } else if (opMode.gamepad2.dpad_left) {
            low();
        } else if (opMode.gamepad2.dpad_down) {
            initial();
        }

        if (opMode.gamepad2.left_stick_y < 0.2) {
            decreaseMotorPosition();
        } else if (opMode.gamepad2.left_stick_y > 0.2) {
            increaseMotorPosition();
        }
    }


    @Override
    public void periodic() {
        telemetry.addData("Lift Motor position", motorPosition);
        telemetry.addData("Lift Servo position", servoPosition);
    }
}