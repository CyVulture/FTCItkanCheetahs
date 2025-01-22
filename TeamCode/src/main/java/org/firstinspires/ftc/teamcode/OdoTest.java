/*   MIT License
 *   Copyright (c) [2024] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

/*
This opmode shows how to use the goBILDA® Pinpoint Odometry Computer.
The goBILDA Odometry Computer is a device designed to solve the Pose Exponential calculation
commonly associated with Dead Wheel Odometry systems. It reads two encoders, and an integrated
system of senors to determine the robot's current heading, X position, and Y position.

it uses an ESP32-S3 as a main cpu, with an STM LSM6DSV16X IMU.
It is validated with goBILDA "Dead Wheel" Odometry pods, but should be compatible with any
quadrature rotary encoder. The ESP32 PCNT peripheral is speced to decode quadrature encoder signals
at a maximum of 40mhz per channel. Though the maximum in-application tested number is 130khz.

The device expects two perpendicularly mounted Dead Wheel pods. The encoder pulses are translated
into mm and their readings are transformed by an "offset", this offset describes how far away
the pods are from the "tracking point", usually the center of rotation of the robot.

Dead Wheel pods should both increase in count when moved forwards and to the left.
The gyro will report an increase in heading when rotated counterclockwise.

The Pose Exponential algorithm used is described on pg 181 of this book:
https://github.com/calcmogul/controls-engineering-in-frc

For support, contact tech@gobilda.com

-Ethan Doak
 */

@Autonomous(name="Odometry Test", group="Linear OpMode")
//@Disabled

public class OdoTest extends LinearOpMode {

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    private DcMotor headLeft;
    private DcMotor headRight;
    private DcMotor shoulderLeft;
    private DcMotor shoulderRight;
    private DcMotor armMotor;
    private DcMotor armSliderMotor;
    private Servo wrist;
    private Servo claw;

    double oldTime = 0;

    double rightOfSubmersibleX = 940;
    double submersiblePosY = -800;
    double pastSamplePosY = -2400;
    double firstSamplePosX = 1300;

    double secondSamplePosX = 1500;

    double thirdSamplePosX = 1800;
    double inObservationZonePosY = -1700;
    double toPickUpSpecimenPosY = -1850;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        headLeft = hardwareMap.get(DcMotor.class, "Bottom Right");
        headRight = hardwareMap.get(DcMotor.class, "Bottom Left");
        shoulderLeft = hardwareMap.get(DcMotor.class, "Top Right");
        shoulderRight = hardwareMap.get(DcMotor.class, "Top Left");
        //armMotor: negative power moves it up, and positive power moves it down
        armMotor = hardwareMap.get(DcMotor.class, "Arm Motor");
        armSliderMotor = hardwareMap.get(DcMotor.class, "Arm Extension Motor");
        wrist = hardwareMap.get(Servo.class, "Wrist");
        claw = hardwareMap.get(Servo.class, "Intake Wheel");
        shoulderRight.setDirection(DcMotor.Direction.REVERSE);
        headRight.setDirection(DcMotor.Direction.REVERSE);
        headLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        headRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulderRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulderLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odo.setOffsets(-84.0, -168.0); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        //odo.setEncoderResolution(13.26291192);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);


        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //odo.recalibrateIMU();
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        double currentPositionX;
        double currentPositionY = odo.getPosY();


        double motorPower = 0.3;

        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);
        odo.setPosition(startingPosition);
        odo.update();
        headLeft.setPower(motorPower);
        headRight.setPower(motorPower);
        shoulderLeft.setPower(motorPower);
        shoulderRight.setPower(motorPower);

        hangSpecimenOne();
        extendSliderMotor();
        moveWristAndClaw();
        fromStartingPositionToSubmsersible();

        move(0);

        currentPositionX = odo.getPosX();
        String data2 = String.format(Locale.US, "{currentPositionX: %.3f}", currentPositionX);
        telemetry.addData("Position after first loop", data2);
        telemetry.update();
        sleep(1000);
        moveRight(0.3);
        odo.update();
//        hangSpecimenOne();

/*After first spike and setting up to put first sample*/
/*   into the submersible */
        afterFirstSpikeToRightOfSubmersible();

        move(0);
        odo.update();
        double driftHeadingDegrees = odo.getPosition().getHeading(AngleUnit.DEGREES);

        fixOrientationNorth(true);

        odo.update();

        setupForSamplePushByMovingForward();

        fixOrientationNorth(true);

        odo.update();
        setupForPushOfSampleByMovingSideways(firstSamplePosX);
//        fixOrientationNorth(true);
        odo.update();
//        turnAround();
        fixOrientationSouth(true);
        move(0);
        fixOrientationSouth(false);
        move(0);

        pushSampleToObservationZone();
        move(0);
        fixOrientationSouth(true);
        fixOrientationSouth(false);
        move(0);
        moveBackToPickUpSpecimen();

//        setupForSamplePushByMovingForward();
//        move(0);
//        fixOrientation(true);
//        fixOrientation(false);
//        move(0);
//        setupForPushOfSampleByMovingSideways(secondSamplePosX);
//        fixOrientation(true);
//        fixOrientation(false);
//        move(0);
//        pushSampleToObservationZone();
//        fixOrientation(true);
//        fixOrientation(false);
//        move(0);
//        setupForSamplePushByMovingForward();
//        fixOrientation(true);
//        fixOrientation(false);
//        move(0);
//        move(0);
//        setupForPushOfSampleByMovingSideways(thirdSamplePosX);
//        move(0);
//        pushSampleToObservationZone();
//        move(0);



        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);
        telemetry.addData("Test", "Successful");
        telemetry.update();
        sleep(15000);
    }
    public void moveWristAndClaw(){
        wrist.setPosition(0.4);
        wrist.setPosition(0.0);
        claw.setPosition(0.5);
        claw.setPosition(1);
    }
    public void hangSpecimenOne() {
        stopMotors();
        armMotor.setPower(-0.5);
        sleep(500);
        stopArm();
    }
    public void extendSliderMotor(){
        stopMotors();
        armSliderMotor.setPower(0.5);
        sleep(500);
        armSliderMotor.setPower(0);
    }
    public void stopArm(){
        armMotor.setPower(0);
        armSliderMotor.setPower(0);
    }


    public void moveBackToPickUpSpecimen(){
        odo.update();
        double currentPosY = odo.getPosY();
        while (opModeIsActive() && currentPosY > toPickUpSpecimenPosY){
            odo.update();
            moveReverse(0.3);
            currentPosY = odo.getPosY();
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("moving back to collect specimen...", data);
            telemetry.update();
        }
        stopMotors();
    }

    public void stopMotors() {
        move(0);
    }
    public void turnAround(){
        odo.update();
        Pose2D pos = odo.getPosition();
        double currentHeading = pos.getHeading(AngleUnit.DEGREES);
        double fullTurn = -179;
        while (opModeIsActive() && currentHeading > fullTurn){
            odo.update();
            rotate(0.3, true);
            pos = odo.getPosition();
            currentHeading = pos.getHeading(AngleUnit.DEGREES);
            String data = String.format(Locale.US, "{degrees: %.3f}", pos.getHeading(AngleUnit.DEGREES));
            data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("doing a full turn...", data);
            telemetry.update();
        }

        stopMotors();
    }
    public void pushSampleToObservationZone() {
        odo.update();
        double currentPosY = odo.getPosY();
        while (opModeIsActive() && currentPosY < inObservationZonePosY){
            odo.update();
            moveForward(0.3);
            currentPosY = odo.getPosY();
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("pushing to observation zone...", data);
            telemetry.update();
        }
        stopMotors();
        sleep(1000);
    }

    public void afterFirstSpikeToRightOfSubmersible() {
        odo.update();
        double currentPositionX = odo.getPosX();

//      setting it up to the position to the right of submsersible
        while (opModeIsActive() && currentPositionX < rightOfSubmersibleX) {
            odo.update();
            currentPositionX = odo.getPosX();
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("after spike moving to right of submersible...", data);
            telemetry.update();
        }
        stopMotors();
    }

    public void setupForSamplePushByMovingForward() {
        odo.update();
        double currentPosY = odo.getPosY();
        while (opModeIsActive() && currentPosY > pastSamplePosY){
            odo.update();
            moveForward(0.3);
            currentPosY = odo.getPosY();
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("setting up for sample push by moving forward...", data);
            telemetry.update();
        }
        stopMotors();
    }


    public void setupForPushOfSampleByMovingSideways(double targetPosX) {
        odo.update();
        double currentPositionX = odo.getPosX();
        while (opModeIsActive() && currentPositionX < targetPosX){
            odo.update();
            moveRight(0.3);
            currentPositionX = odo.getPosX();
        }
        stopMotors();
    }


    public void fromStartingPositionToSubmsersible() {
        odo.update();
        double currentPositionY = odo.getPosY();
        while (opModeIsActive() && currentPositionY > submersiblePosY) {
            odo.update();

            currentPositionY = odo.getPosY();

            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;


            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            telemetry.update();

        }
        stopMotors();
    }
    public void setMotorDirection(DcMotorSimple.Direction direction){
        shoulderRight.setDirection(direction);
        headRight.setDirection(direction);
    }

    public void move(double motorPower, int direction, boolean accountForDrift) {

        double fixForSlowness = 0.1;
        headLeft.setPower(motorPower);
        headRight.setPower(motorPower);
        shoulderLeft.setPower(motorPower);
        shoulderRight.setPower(motorPower*(accountForDrift ? fixForSlowness : 1));
    }

    public void moveRight(double motorPower, boolean accountForDrift) {
        setMotorDirection(DcMotor.Direction.FORWARD);
        double fixForSlowness = 0.1;
        headLeft.setPower(motorPower);
        headRight.setPower(motorPower);
        shoulderLeft.setPower(-motorPower);
        shoulderRight.setPower(-motorPower*(accountForDrift ? fixForSlowness : 1));
    }

    public void moveRight(double motorPower) {
        moveRight(motorPower, false);
    }

    public void moveLeft(double motorPower) {
        setMotorDirection(DcMotor.Direction.FORWARD);
        moveRight(-motorPower, false);
    }

    public void move(double motorPower) {
        move(motorPower, 1, false);
    }

    public void moveForward(double motorPower) {
        setMotorDirection(DcMotor.Direction.REVERSE);
        move(motorPower);
    }
    public void moveReverse(double motorPower){
        moveForward(-motorPower);
    }


    public void rotate(double motorPower, boolean isRight){
        setMotorDirection(DcMotor.Direction.REVERSE);
        headRight.setPower( (isRight ? -1 : 1) * motorPower);
        headLeft.setPower((isRight ? 1 : -1) * motorPower);
        shoulderRight.setPower((isRight ? -1 : 1) * motorPower);
        shoulderLeft.setPower((isRight ? 1 : -1) * motorPower);
    }
    public void fixOrientationSouth(boolean isDriftNegativeDegrees) {
        odo.update();
        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{degrees: %.3f}", pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("fixing orientation south by rotating ...", data);
        telemetry.update();
        sleep(1000);
        double driftHeadingDegrees = pos.getHeading(AngleUnit.DEGREES);

        if (isDriftNegativeDegrees) {
            while (opModeIsActive() && driftHeadingDegrees < 170 && driftHeadingDegrees > 0){
                odo.update();
                pos = odo.getPosition();
                rotate(0.2, false);
                double angularVelocity = odo.getHeadingVelocity();

                data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f, HV: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES), angularVelocity);
                telemetry.addData("fixing orientation by rotating left...", data);
                telemetry.update();
                driftHeadingDegrees = pos.getHeading(AngleUnit.DEGREES);
            }
        } else {

            while (opModeIsActive() && driftHeadingDegrees > -176 && driftHeadingDegrees < 0){
                odo.update();
                pos = odo.getPosition();
                rotate(0.2, true);
                double angularVelocity = odo.getHeadingVelocity();
                data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f, HV: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES), angularVelocity);
                telemetry.addData("fixing orientation by rotating right...", data);
                telemetry.update();
                driftHeadingDegrees = pos.getHeading(AngleUnit.DEGREES);
            }

}
    }

    public void fixOrientationNorth(boolean isDriftNegativeDegrees) {
        odo.update();
        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{degrees: %.3f}", pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("fixing orientation by rotating ...", data);
        telemetry.update();
        double driftHeadingDegrees = pos.getHeading(AngleUnit.DEGREES);

        if (isDriftNegativeDegrees) {
            while (opModeIsActive() && driftHeadingDegrees < -3 && driftHeadingDegrees > -178){
                odo.update();
                pos = odo.getPosition();
                rotate(0.2, false);
                double angularVelocity = odo.getHeadingVelocity();

                data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f, HV: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES), angularVelocity);
                telemetry.addData("fixing orientation by rotating right...", data);
                telemetry.update();
                driftHeadingDegrees = pos.getHeading(AngleUnit.DEGREES);
            }
        } else {

            while (opModeIsActive() && driftHeadingDegrees > 3 && driftHeadingDegrees < 178){
                odo.update();
                pos = odo.getPosition();
                rotate(0.2, true);
                double angularVelocity = odo.getHeadingVelocity();
                data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f, HV: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES), angularVelocity);
                telemetry.addData("fixing orientation by rotating left...", data);
                telemetry.update();
                driftHeadingDegrees = pos.getHeading(AngleUnit.DEGREES);
            }

        }



    }





}


