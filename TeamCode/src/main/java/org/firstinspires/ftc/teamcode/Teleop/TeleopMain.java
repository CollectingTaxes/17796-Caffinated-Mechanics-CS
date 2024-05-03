package org.firstinspires.ftc.teamcode.Teleop;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.subsystem.Slides;
import org.firstinspires.ftc.teamcode.subsystem.TelemetrySubsystem;
import org.firstinspires.ftc.teamcode.subsystem.Wheel;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.CommandSchedulerWrapper;

import java.util.function.BooleanSupplier;

@TeleOp
public class TeleopMain extends CommandOpMode {
    @Override
    public void initialize() {
        // TODO: Move to Robot Container
        BetterGamepad driver = new BetterGamepad(gamepad1);
        BetterGamepad operator = new BetterGamepad(gamepad2);

        CommandSchedulerWrapper command = new CommandSchedulerWrapper();

        Drive drive = new Drive(this);

        Slides lift = new Slides(this);
        Wheel wheel = new Wheel(this);


        TelemetrySubsystem telemetrySubsystem = new TelemetrySubsystem(
                telemetry,
                drive,
                lift);

        command.addDefault(() -> telemetrySubsystem.periodic(driver, operator));

        /*
         *
         * DRIVER COMMANDS
         *
         */

        command.addDefault(() -> drive.drive(
                driver.getLeftX(), driver.getLeftY(), driver.getRightX(), DriveConstants.Drivetrain.Value.FINE_CONTROL, DriveConstants.Drivetrain.Value.FIELD_CENTRIC));

        command.add(() -> driver.get(RIGHT_BUMPER))
                .whenPressed(drive::setSlow)
                .whenReleased(drive::setNormal);

        command.add(() -> driver.get(LEFT_BUMPER))
                .whenPressed(drive::setTurbo)
                .whenReleased(drive::setNormal);

        command.add(() -> driver.get(A))
                .whenPressed(drive::resetImu);

        command.add(() -> driver.getTriggerPressed(LEFT_TRIGGER))
                .whileActiveContinuous(wheel::open)
                        .whenActive(wheel::off);

        command.add(() -> driver.getTriggerPressed(RIGHT_TRIGGER))
                .whileActiveContinuous(wheel::close)
                        .whenInactive(wheel::off);

        /*
         *
         * OPERATOR COMMANDS
         *
         */

        command.add(() -> operator.getLeftY() > 0.3)
                .whileActiveContinuous(lift::increaseMotorPosition)
                        .whenInactive(lift::off);

        command.add(() -> operator.getLeftY() < -0.3)
                .whileActiveContinuous(lift::decreaseMotorPosition)
                        .whenInactive(lift::off);

        command.add(() -> operator.getRightY() > 0.3)
                .whileHeld(lift::increaseServoPosition);

        command.add(() -> operator.getRightY() < -0.3)
                .whileHeld(lift::decreaseServoPosition);
    }
    // There are two things that get run when you do this.
    // The periodic method of all defined subsystems, and
    // the runnable used on the all of the buttons.
    // This runnable will be active based on the get function of
    // the trigger which is why you have to override the get
    // method of Button to be able to use it
}