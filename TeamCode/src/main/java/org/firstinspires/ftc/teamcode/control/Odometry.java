package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class Odometry {

    private double x = 0.0;
    private double y = 0.0;
    private double theta = 0.0;
    private double accumulatedAngle = 0.0;

    private final DcMotor leftMotor;
    private final DcMotor rightMotor;
    private final IMU imu;
    private final Drivetrain drivetrain;
    private final LinearOpMode opMode;

    private final double wheelRadius;
    private final double ticksPerRevolution;

    private Integer lastLeftTicks = null;
    private Integer lastRightTicks = null;
    private double lastTheta = 0.0;

    public Odometry(
            DcMotor leftMotor,
            DcMotor rightMotor,
            IMU imu,
            Drivetrain drivetrain,
            LinearOpMode opMode,
            double wheelRadius,
            double ticksPerRevolution
    ) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.imu = imu;
        this.drivetrain = drivetrain;
        this.opMode = opMode;
        this.wheelRadius = wheelRadius;
        this.ticksPerRevolution = ticksPerRevolution;
    }

    private double getImuTheta() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.RADIANS);
    }

    private double degreesToRadians(double angleDeg) {
        return Math.toRadians(angleDeg);
    }

    public void update() {
        int leftTicks = leftMotor.getCurrentPosition();
        int rightTicks = rightMotor.getCurrentPosition();
        double currentTheta = getImuTheta();

        if (lastLeftTicks == null || lastRightTicks == null) {
            lastLeftTicks = leftTicks;
            lastRightTicks = rightTicks;
            theta = currentTheta;
            lastTheta = currentTheta;
            return;
        }

        int deltaLeftTicks = leftTicks - lastLeftTicks;
        int deltaRightTicks = rightTicks - lastRightTicks;

        double leftTurns = deltaLeftTicks / ticksPerRevolution;
        double rightTurns = deltaRightTicks / ticksPerRevolution;

        double dLeft = leftTurns * 2.0 * Math.PI * wheelRadius;
        double dRight = rightTurns * 2.0 * Math.PI * wheelRadius;
        double distance = (dLeft + dRight) / 2.0;

        double deltaTheta = angleDifferenceRad(currentTheta, lastTheta);
        accumulatedAngle = 0.0;
        double thetaMid = lastTheta + deltaTheta / 2.0;

        x += distance * Math.cos(thetaMid);
        y += distance * Math.sin(thetaMid);
        theta = currentTheta;

        lastLeftTicks = leftTicks;
        lastRightTicks = rightTicks;
        lastTheta = currentTheta;
    }

    public double angleDifferenceRad(double currentAngle, double startAngle) {
        double difference = currentAngle - startAngle;

        while (difference > Math.PI) {
            difference -= 2.0 * Math.PI;
        }

        while (difference < -Math.PI) {
            difference += 2.0 * Math.PI;
        }

        return difference;
    }

    public void turnLeft(double targetAngleDeg) {
        accumulatedAngle = 0.0;
        update();

        while (opMode.opModeIsActive()) {
            drivetrain.setPower(-0.40, 0.40);
            update();

            if (accumulatedAngle >= degreesToRadians(targetAngleDeg)) {
                break;
            }
        }

        drivetrain.stop();
    }

    public void turnRight(double targetAngleDeg) {
        accumulatedAngle = 0.0;
        update();

        while (opMode.opModeIsActive()) {
            drivetrain.setPower(0.40, -0.40);
            update();

            if (accumulatedAngle <= -degreesToRadians(targetAngleDeg)) {
                break;
            }
        }

        drivetrain.stop();
    }

    public void driveDistance(double targetDistance) {
        update();

        double startX = x;
        double startY = y;

        while (opMode.opModeIsActive()) {
            drivetrain.setPower(0.30, 0.30);
            update();

            double deltaX = x - startX;
            double deltaY = y - startY;
            double travelledDistance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

            if (travelledDistance >= targetDistance) {
                break;
            }
        }

        drivetrain.stop();
    }

    public void reset() {
        x = 0.0;
        y = 0.0;
        theta = 0.0;
        accumulatedAngle = 0.0;

        imu.resetYaw();

        lastTheta = 0.0;
        lastLeftTicks = null;
        lastRightTicks = null;
    }

    public static class Pose {
        public final double x;
        public final double y;
        public final double thetaRad;
        public final double thetaDeg;

        public Pose(double x, double y, double theta) {
            this.x = x;
            this.y = y;
            this.thetaRad = theta;
            this.thetaDeg = Math.toDegrees(theta);
        }
    }

    public Pose getPose() {
        return new Pose(x, y, theta);
    }
}