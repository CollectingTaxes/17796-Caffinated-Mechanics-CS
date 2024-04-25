package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.subsystem.Slides;
import org.firstinspires.ftc.teamcode.subsystem.Wheel;
import org.firstinspires.ftc.teamcode.subsystem.Wrist;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

public class TeleopMain extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Slides lift = new Slides(this);
        Wheel wheel = new Wheel(this);
        Drive drive = new Drive(this, true);
        Wrist wrist = new Wrist(this);

        drive.setPoseEstimate(PoseStorage.currentPose);
        wrist.reset();
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            lift.runIteratively();
            wheel.runIteratively();

            drive.runIteratively();
            telemetry.update();
        }

    }
}
