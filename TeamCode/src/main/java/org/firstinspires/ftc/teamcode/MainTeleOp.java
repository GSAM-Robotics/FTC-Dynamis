package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intaker;

@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends OpMode {

    private Drivetrain drivetrain;
    private Intaker intaker;
    private boolean activeIntaker = false;
    private boolean lastRb = false;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        intaker = new Intaker(hardwareMap);

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
            intaker.ControlIntaker(1.0);
        } else {
            intaker.stop();
        }

        telemetry.addData("Drive", drive);
        telemetry.addData("Turn", turn);
        telemetry.addData("Intake", gamepad1.right_bumper);
        telemetry.update();
    }

    @Override
    public void stop() {
        drivetrain.stop();
        intaker.stop();
    }
}