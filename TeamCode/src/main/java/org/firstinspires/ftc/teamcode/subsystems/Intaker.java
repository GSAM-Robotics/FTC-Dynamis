package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intaker {

    private final DcMotor torqueMotor;

    public Intaker(HardwareMap hardwareMap) {
        torqueMotor = hardwareMap.get(DcMotor.class, "torqueMotor");
        torqueMotor.setDirection(DcMotor.Direction.REVERSE);
        torqueMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        torqueMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void ControlIntaker(double power) {
        torqueMotor.setPower(power);
    }

    public void outtake(double power) {
        torqueMotor.setPower(-power);
    }

    public void stop() {
        torqueMotor.setPower(0);
    }
}