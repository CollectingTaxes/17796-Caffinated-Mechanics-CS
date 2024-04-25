package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class MotorExEx extends MotorEx {

    // Overridden field
    protected PIDFController positionController = new PIDFController(1, 0, 0, 0);


    /**
     * Constructs an instance motor for the wrapper wrapper
     *
     * @param hMap the hardware map from the OpMode
     * @param id   the device id from the RC config
     * @param cpr  the counts per revolution of the motor
     * @param rpm  the revolutions per minute of the motor
     */

    public MotorExEx(@NonNull HardwareMap hMap, String id, double cpr, double rpm) {
        super(hMap, id, cpr, rpm);
        motorEx = (DcMotorEx) super.motor;
        encoder.reset();
    }


    public void setTargetDegrees(double degrees) {
        setTargetPosition((int) (getCPR() * degrees / 360));
    }

    @Override
    public void set(double output) {
        if (runmode == RunMode.VelocityControl) {
            double speed = bufferFraction * output * ACHIEVABLE_MAX_TICKS_PER_SECOND;
            double velocity = veloController.calculate(getVelocity(), speed) + feedforward.calculate(speed, encoder.getAcceleration());
            motor.setPower(velocity / ACHIEVABLE_MAX_TICKS_PER_SECOND);
        } else if (runmode == RunMode.PositionControl) {
            double error = positionController.calculate(getDistance());
            motor.setPower(output * error);
        } else {
            motor.setPower(output);
        }
    }

    public void setVeloCoefficients(double kp, double ki, double kd, double Kf) {
        veloController.setPIDF(kp, ki, kd, 0);
    }

    public void setVeloCoefficients(PIDFCoefficients coefficients) {
        veloController.setPIDF(coefficients.p, coefficients.i, coefficients.d, coefficients.f);
    }

    public void setPositionCoefficients(PIDFCoefficients coefficients) {
        positionController.setPIDF(coefficients.p, coefficients.i, coefficients.d, coefficients.f);
    }
    public MotorConfigurationType getMotorType() {
        return motor.getMotorType();
    }
    public void setMotorType(MotorConfigurationType motorConfigurationType) {
        motor.setMotorType(motorConfigurationType);
    }
}