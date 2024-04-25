package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.subsystem.Wrist.WristConstants.hardware;
import static org.firstinspires.ftc.teamcode.subsystem.Wrist.WristConstants.value;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Wrist extends HardwareSubsystem {

    private final ServoEx leftServo;
    private final ServoEx rightServo;

    public static class WristConstants {
        public static Hardware hardware = new Hardware();
        public static Value value = new Value();

        public static class Hardware {
            public String ID = "leftServo";
            public String Id = "rightServo";
            public double MIN_ANGLE = 0;
            public double MAX_ANGLE = 300;
            public boolean REVERSED = true;
        }
        public static class Value {
            public double UP = 220; //degrees
            public double DOWN = 50; //degrees
        }
    }

    public Wrist(OpMode opMode) {
        super(opMode);
        leftServo = new SimpleServo(hardwareMap, hardware.ID, hardware.MIN_ANGLE, hardware.MAX_ANGLE);
        leftServo.setInverted(hardware.REVERSED);

        rightServo = new SimpleServo(hardwareMap, hardware.Id, hardware.MIN_ANGLE, hardware.MAX_ANGLE);

    }

    public void reset(){leftServo.turnToAngle(WristConstants.value.UP);
                        rightServo.turnToAngle(value.UP);}

    public void score(){leftServo.turnToAngle(WristConstants.value.DOWN);
                        rightServo.turnToAngle(value.DOWN);}

    @Override
    public void init() {

    }
    @Override
    public void periodic() {

    }
}