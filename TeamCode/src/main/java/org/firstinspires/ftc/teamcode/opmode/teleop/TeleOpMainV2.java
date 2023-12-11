package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.subsystem.DroneLauncher;
import org.firstinspires.ftc.teamcode.subsystem.HangingMech;
import org.firstinspires.ftc.teamcode.subsystem.Slides;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
@TeleOp
public class TeleOpMainV2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(this);
        //Drive code is all the way at the bottom of the Drive class
        Slides slides = new Slides(this);
        Arm arm = new Arm(this);
        DroneLauncher droneLauncher = new DroneLauncher(this);
        Intake intake = new Intake(this);
        HangingMech hangingMech = new HangingMech(this);
        Claw claw = new Claw(this);

        waitForStart();
        while (opModeIsActive()) {
            arm.teleOp();
            drive.teleOp();
            slides.teleOp();
            droneLauncher.teleOp();
            intake.teleOp();
            hangingMech.teleOp();
            claw.teleOp();
        }
    }
}
