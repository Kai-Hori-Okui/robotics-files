package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class JTracking {
    private LinearOpMode opMode;
    private HardwareMap hardwareMap;

    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor arm;
    private Servo claw;

    private Telemetry telemetryA;
    private SparkFunOTOS otos;

    // tune these values to the point where moveFieldCentric(1, 1, 0, 0.2, 0) moves it exactly diagonally to the top-right
    final double forwardFactor = 1.0;
    final double strafeFactor = 1.0;
    final double posErrorTolerance = 0.05;
    final double headingErrorTolerance = 1.0;

    final double position_p = 0.04;
    final double position_d = 0.02;

    final double heading_p = 0.03;
    final double heading_d = 0.02;

    // we add minimum powers to prevent it from halting and never reaching the target.
    // at 0.1 movement power, it can pretty much stop as soon as it wants to.
    final double maxPower = 0.80;
    final double minPower = 0.10;

    final double maxYaw = 0.40;
    final double minYaw = 0.05;

    public JTracking(LinearOpMode initOpMode, HardwareMap initHardwareMap) {
        opMode = initOpMode;
        hardwareMap = initHardwareMap;


        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        otos.setOffset(new SparkFunOTOS.Pose2D(0.075, 3.40625, 0));
        otos.setLinearScalar(1.0739); //0.9637
        otos.setAngularScalar(0.9798);

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);

        otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
        otos.resetTracking();

        telemetryA = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addData("Status", "Initialized");
        telemetryA.update();
    }

    // Coordinate system: (0,0) is the center of the 4 central tiles, 1 unit is 1 inch
    // angle of 0 is pointing in +X direction, angles increase COUNTERCLOCKWISE

    public void setMotors(double bl, double br, double fl, double fr) {
        backLeft.setPower(bl);
        backRight.setPower(br);
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
    }

    public void stopMotors() {
        setMotors(0,0,0,0);
    }

    public void setMotorsOmni(double axial, double lateral, double yaw) {
        // again, a positive yaw makes the robot rotate COUNTERCLOCKWISE
        setMotors(axial - lateral - yaw, axial + lateral + yaw, axial + lateral - yaw, axial - lateral + yaw);
    }

    public void moveFieldCentric(double dx, double dy, double heading, double power, double yaw) {
        // strafing often requires more power than going forward, so tweaking the forward and strafe factors may be necessary.
        double scaledX = dx * forwardFactor;
        double scaledY = dy * strafeFactor;

        double d = Math.sqrt(scaledX*scaledX + scaledY*scaledY);

        // if you think about what should happen when we have a heading of 0, this should make sense.
        double normalizedX = scaledX / d;
        double normalizedY = scaledY / d;

        // we initialize
        double newLateral = normalizedX * Math.sin(heading / 180 * Math.PI) - normalizedY * Math.cos(heading / 180 * Math.PI);
        double newAxial = normalizedX * Math.cos(heading / 180 * Math.PI) + normalizedY * Math.sin(heading / 180 * Math.PI);

        newAxial *= power;
        newLateral *= power;

        setMotorsOmni(newAxial, newLateral, yaw);
    }

    public double getError(double targetX, double targetY, double currentX, double currentY) {
        double errorX = targetX - currentX;
        double errorY = targetY - currentY;

        return Math.sqrt(errorX*errorX + errorY*errorY);
    }

    public double minimizeAngle(double angle) {
        return ((angle + 180.0) % 360.0) - 180.0;
    }

    public constrainPower(double min, double max, double power) {
        double absPower = Math.abs(power);
        double sign = Math.signum(power);

        return sign * Math.min(Math.max(absPower, min), max);
    }

    public void moveTo(double targetX, double targetY, double targetHeading) {
        SparkFunOTOS.Pose2D pos = otos.getPosition();

        double yaw = 0;
        double power = 0;
        double errorX = targetX - pos.x;
        double errorY = targetY - pos.y;

        double posError = Math.sqrt(errorX*errorX + errorY*errorY);
        double prevPosError = posError;
        double posErrorDiff = 0;

        double headingError = minimizeAngle(targetHeading - pos.h);
        double prevHeadingError = headingError;
        double headingErrorDiff = 0;

        while (opMode.opModeIsActive() && ((posError > posErrorTolerance) || (Math.abs(headingError) > headingErrorTolerance))) {
            prevPosError = posError;
            prevHeadingError = headingError;

            // recalculate position and error
            pos = otos.getPosition();

            errorX = targetX - pos.x;
            errorY = targetY - pos.y;

            posError = Math.sqrt(errorX*errorX + errorY*errorY);
            headingError = minimizeAngle(targetHeading - pos.h);

            posErrorDiff = posError - prevPosError;
            headingErrorDiff = headingError - prevHeadingError;

            // remember: positive yaw means rotation COUNTERCLOCKWISE
            yaw = heading_p * headingError + heading_d * headingErrorDiff;
            yaw = constrainPower(minYaw, maxYaw, yaw);

            power = position_p * posError + position_d * posErrorDiff;
            power = constrainPower(minPower, maxPower, power);

            // using error x and y, we know the exact direction we need to travel in the x and y directions
            moveFieldCentric(errorX, errorY, pos.h, power, yaw);
        }

        stopMotors();
    }
}