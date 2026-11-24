package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import org.firstinspires.ftc.teamcode.control.Odometry;

@Autonomous(name = "MainAutonomous")
public class MainAutonomous extends LinearOpMode {

    private Drivetrain drivetrain;
    private Odometry odometry;

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private IMU imu;

    @Override
    public void runOpMode() {

        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        imu = hardwareMap.get(IMU.class, "imu");

        drivetrain = new Drivetrain(hardwareMap);

        odometry = new Odometry(
                leftMotor,
                rightMotor,
                imu,
                drivetrain,
                this,
                0.045,
                560.0
        );

        odometry.reset();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) {
            return;
        }

        odometry.update();

        odometry.turnLeft(360);



        drivetrain.stop();
    }
}