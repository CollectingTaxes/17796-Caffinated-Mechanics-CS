package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.subsystem.Slides;
import org.firstinspires.ftc.teamcode.subsystem.Wheel;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@TeleOp
public class TeleopMain extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Slides lift = new Slides(this);
        Wheel wheel = new Wheel(this);
        Drive drive = new Drive(this, true);

        drive.setPoseEstimate(PoseStorage.currentPose);
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            wheel.runIteratively();

            drive.runIteratively();
            telemetry.update();
        }

    }
}
