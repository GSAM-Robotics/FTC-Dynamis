
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends OpMode {

    private Drivetrain drivetrain;
    private CRServo servoLeft;
    private CRServo servoRight;

    private boolean activeIntaker = false;
    private boolean lastRb = false;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);

        servoLeft = hardwareMap.get(CRServo.class, "servoLeft");
        servoRight = hardwareMap.get(CRServo.class, "servoRight");

        servoLeft.setPower(0.0);
        servoRight.setPower(0.0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        double drive = -gamepad1.right_stick_y;
        double turn = gamepad1.left_stick_x;
        drivetrain.arcadeDrive(drive, turn);

        boolean currentRb = gamepad1.right_bumper;

        if (currentRb && !lastRb) {
            activeIntaker = !activeIntaker;
        }
        lastRb = currentRb;

        if (activeIntaker) {
            servoLeft.setPower(1.0);
            servoRight.setPower(1.0);
        } else {
            servoLeft.setPower(0.0);
            servoRight.setPower(0.0);
        }

        telemetry.addData("Drive", drive);
        telemetry.addData("Turn", turn);
        telemetry.addData("Intake", activeIntaker);
        telemetry.update();
    }

    @Override
    public void stop() {
        drivetrain.stop();
        servoLeft.setPower(0.0);
        servoRight.setPower(0.0);
    }
}
