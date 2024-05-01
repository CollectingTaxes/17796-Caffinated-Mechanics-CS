package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.util.DriveConstants.Controller.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.Controller.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.Controller.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.Controller.MAX_VEL;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.Controller.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.Controller.RUN_USING_BUILT_IN_CONTROLLER;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.Controller.kA;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.Controller.kStatic;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.Controller.kV;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.Drivetrain.Value.TELEOP_NORMAL;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.Drivetrain.Value.TELEOP_SLOWER;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.Drivetrain.Value.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.Follower.HEADING_PID;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.Follower.INITIAL_POS_MAYBE;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.Follower.LATERAL_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.Follower.OMEGA_WEIGHT;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.Follower.TIMEOUT;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.Follower.TRANSLATIONAL_PID;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.Follower.VX_WEIGHT;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.Follower.VY_WEIGHT;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.Imu.Hardware.ID;
import static org.firstinspires.ftc.teamcode.util.DriveConstants.encoderTicksToInches;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.util.DriveConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.AxisDirection;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.util.MotorExEx;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class Drive extends MecanumDrive implements Subsystem {

    /** FTC Lib Subsystem stuff (this stuff is defined in SubsystemBase,
     * but since we are already extending a class, we cant extend it
     * we have to do this instead
     */

    protected String m_name = this.getClass().getSimpleName();

    public String getName() {
        return m_name;
    }

    public void setName(String name) {
        m_name = name;
    }

    public String getSubsystem() {
        return getName();
    }

    public void setSubsystem(String subsystem) {
        setName(subsystem);
    }

    private double multiplier = 1;



    private final TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private final MotorExEx leftFront, leftRear, rightRear, rightFront;
    private final List<MotorExEx> motors;

    private final BNO055IMU imu;
    private final VoltageSensor batteryVoltageSensor;

    private final LinearOpMode opMode;

    public static double ENCODER_MULTIPLIER = 1; // Multiplier for drive encoders (might not be that necesary)

    // This is to make an FtcLib mecanum drive
//    com.arcrobotics.ftclib.drivebase.MecanumDrive controllerMecanumDrive;

    public Drive(LinearOpMode opMode) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);
        this.opMode = opMode;
        HardwareSubsystem.initializeConstants();

        CommandScheduler.getInstance().registerSubsystem(this);


        TrajectoryFollower follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                INITIAL_POS_MAYBE, TIMEOUT);

        LynxModuleUtil.ensureMinimumFirmwareVersion(opMode.hardwareMap);

        batteryVoltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : opMode.hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = opMode.hardwareMap.get(BNO055IMU.class, ID);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        //TODO: this will probably mess up roadrunner put it back to radians or maybe
        // do math.toDegrees
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: If the hub containing the IMU you are using is mounted so that the "REV" logo does
        // not face up, remap the IMU axes so that the z-axis points upward (normal to the floor.)
        //
        //             | +Z axis
        //             |
        //             |
        //             |
        //      _______|_____________     +Y axis
        //     /       |_____________/|__________
        //    /   REV / EXPANSION   //
        //   /       / HUB         //
        //  /_______/_____________//
        // |_______/_____________|/
        //        /
        //       / +X axis
        //
        // This diagram is derived from the axes in section 3.4 https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
        // and the placement of the dot/orientation from https://docs.revrobotics.com/rev-control-system/control-system-overview/dimensions#imu-location
        //
        // For example, if +Y in this diagram faces downwards, you would use AxisDirection.NEG_Y.
//         BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_Y);

        leftFront = new MotorExEx(opMode.hardwareMap, DriveConstants.Drivetrain.LeftFront.hardware.ID, DriveConstants.Drivetrain.Value.TICKS_PER_REV, DriveConstants.Drivetrain.Value.MAX_RPM);
        leftRear = new MotorExEx(opMode.hardwareMap, DriveConstants.Drivetrain.LeftRear.hardware.ID, DriveConstants.Drivetrain.Value.TICKS_PER_REV, DriveConstants.Drivetrain.Value.MAX_RPM);
        rightFront = new MotorExEx(opMode.hardwareMap, DriveConstants.Drivetrain.RightFront.hardware.ID, DriveConstants.Drivetrain.Value.TICKS_PER_REV, DriveConstants.Drivetrain.Value.MAX_RPM);
        rightRear = new MotorExEx(opMode.hardwareMap, DriveConstants.Drivetrain.RightRear.hardware.ID, DriveConstants.Drivetrain.Value.TICKS_PER_REV, DriveConstants.Drivetrain.Value.MAX_RPM);


        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);


        setAchieveableMaxRPMFraction(1);

        if (RUN_USING_BUILT_IN_CONTROLLER) {
            setMode(Motor.RunMode.VelocityControl);
        }

        setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_BUILT_IN_CONTROLLER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(Motor.RunMode.VelocityControl, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        leftFront.setInverted(DriveConstants.Drivetrain.LeftFront.hardware.REVERSED);
        leftRear.setInverted(DriveConstants.Drivetrain.LeftRear.hardware.REVERSED);
        rightFront.setInverted(DriveConstants.Drivetrain.RightFront.hardware.REVERSED);
        rightRear.setInverted(DriveConstants.Drivetrain.RightRear.hardware.REVERSED);

//        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
//        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        // This is to give our ftc MecanumDrive a value
        // we do this after inverting the motors
//        controllerMecanumDrive = new com.arcrobotics.ftclib.drivebase.MecanumDrive(true,
//                leftFront,
//                leftRear,
//                rightFront,
//                rightRear
//        );

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
//        setLocalizer(new TwoWheelTrackingLocalizer(opMode.hardwareMap, this));

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL,
                opMode

        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(Motor.RunMode runMode) {
        for (MotorExEx motor : motors) {
            motor.setRunMode(runMode);
        }
    }

    public void setZeroPowerBehavior(Motor.ZeroPowerBehavior zeroPowerBehavior) {
        for (MotorExEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(Motor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (MotorExEx motor : motors) {
            switch (runMode){
                case VelocityControl: motor.setVeloCoefficients(compensatedCoefficients);
                case PositionControl: motor.setPositionCoefficients(compensatedCoefficients);
                default: return;
            }

//            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() { // left front, left rear, right front, right rear
        List<Double> wheelPositions = new ArrayList<>();
        for (MotorExEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()) * ENCODER_MULTIPLIER);
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (MotorExEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()) * ENCODER_MULTIPLIER);
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.set(v);
        leftRear.set(v1);
        rightRear.set(v2);
        rightFront.set(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    public void resetImu() {
        imu.initialize(new BNO055IMU.Parameters());
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return (double) imu.getAngularVelocity().xRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    public void driveFieldCentric(double strafeSpeed, double forwardSpeed,
                                  double turnSpeed, double gyroAngle) {
        strafeSpeed = clipRange(strafeSpeed);
        forwardSpeed = clipRange(forwardSpeed);
        turnSpeed = clipRange(turnSpeed);

        Vector2d input = new Vector2d(strafeSpeed, forwardSpeed);
        input = input.rotateBy(-gyroAngle);

        double theta = input.angle();

        double[] wheelSpeeds = new double[4];
        wheelSpeeds[0] = Math.sin(theta + Math.PI / 4);
        wheelSpeeds[1] = Math.sin(theta - Math.PI / 4);
        wheelSpeeds[2] = Math.sin(theta - Math.PI / 4);
        wheelSpeeds[3] = Math.sin(theta + Math.PI / 4);

        normalize(wheelSpeeds, input.magnitude());

        wheelSpeeds[0] += turnSpeed;
        wheelSpeeds[1] -= turnSpeed;
        wheelSpeeds[2] += turnSpeed;
        wheelSpeeds[3] -= turnSpeed;

        normalize(wheelSpeeds);

        leftFront
                .set(wheelSpeeds[0]);
        rightFront
                .set(wheelSpeeds[1]);
        leftRear
                .set(wheelSpeeds[2]);
        rightRear
                .set(wheelSpeeds[3]);
    }
    public void setAchieveableMaxRPMFraction (double fraction) {
        for (MotorExEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(fraction);
            motor.setMotorType(motorConfigurationType);
        }
    }

    public void driveRobotCentric(double x, double y, double rotate, boolean fineControl) {
//        controllerMecanumDrive.driveRobotCentric(x, y, rotate, fineControl);
        setWeightedDrivePower(
                new Pose2d(
                        (y),
                        (-x),
                        (-rotate)
                )
        );

    }

    public void drive(double x, double y, double rotate, boolean fineControl, boolean fieldCentric) {
        x = x * multiplier;
        y = y * multiplier;
        rotate = rotate * multiplier;
        if (fieldCentric) driveFieldCentric(x, y, rotate, Math.toDegrees(getRawExternalHeading()));
        else driveRobotCentric(x, y, rotate, fineControl);
    }

    public void setSlow() {
        multiplier = TELEOP_SLOWER;
//        motors.get(0).set
//        controllerMecanumDrive.setMaxSpeed(DriveConstants.Drivetrain.Value.TELEOP_SLOW);
    }

    public void setTurbo() {
        setAchieveableMaxRPMFraction(DriveConstants.Drivetrain.Value.TELEOP_SLOW);
//        controllerMecanumDrive.setMaxSpeed(DriveConstants.Drivetrain.Value.TELEOP_TURBO);
    }

    public void setNormal() {
        multiplier = TELEOP_NORMAL;
//        controllerMecanumDrive.setMaxSpeed(DriveConstants.Drivetrain.Value.TELEOP_NORMAL);
    }

    public void initTelemetry() {

    }
    public void periodicTelemetry() {
        opMode.telemetry.addData("Drive", "Field centric: " + (DriveConstants.Drivetrain.Value.FIELD_CENTRIC ? "on" : "off"), "Fine Control: " + (DriveConstants.Drivetrain.Value.FINE_CONTROL ? "on" : "off"));
    }

    public double clipRange(double value) {
        return value <= -1 ? -1
                : value >= 1 ? 1
                : value;
    }

    protected void normalize(double[] wheelSpeeds, double magnitude) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        for (int i = 0; i < wheelSpeeds.length; i++) {
            wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude) * magnitude;
        }

    }

    protected void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        if (maxMagnitude > 1) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude);
            }
        }

    }
}