package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


@TeleOp(name = "Autonomous test code", group = "Linear OpMode")
//@Disabled
public class AutonomousSensor extends LinearOpMode {

    GoBildaPinpointDriver odo; // Declare OpModce member for the Odometry Computer

    DcMotor TopLeft;
    DcMotor TopRight;
    DcMotor BottomLeft;
    DcMotor BottomRight;

    @Override
    public void runOpMode() {

        // Initialize hardware
        odo = hardwareMap.get(org.firstinspires.ftc.teamcode.GoBildaPinpointDriver.class, "odo");
        TopLeft = hardwareMap.get(DcMotor.class, "TopLeft");
        TopRight = hardwareMap.get(DcMotor.class, "TopRight");
        BottomLeft = hardwareMap.get(DcMotor.class, "BottomLeft");
        BottomRight = hardwareMap.get(DcMotor.class, "BottomRight");

        // Reverse direction of motors as needed
        TopLeft.setDirection(DcMotor.Direction.REVERSE);
        BottomLeft.setDirection(DcMotor.Direction.REVERSE);
        TopRight.setDirection(DcMotor.Direction.FORWARD);
        BottomRight.setDirection(DcMotor.Direction.FORWARD);

        // Set offsets for odometry pods
        odo.setOffsets(-84.0, -168.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Reset position and calibrate IMU
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Define target position in mm
        double targetX = 1000.0; // Move to 1000 mm (1 meter) forward
        double targetY = 500.0;  // Move to 500 mm (0.5 meters) to the left
        double targetHeading = 0.0; // Maintain a 0-degree heading

        // Movement tolerances
        double positionTolerance = 10.0; // 10 mm
        double headingTolerance = 2.0;   // 2 degrees

        // Proportional control gains
        double kP_Position = 0.01; // For position control
        double kP_Heading = 0.01;  // For heading control

        while (opModeIsActive()) {
            // Update odometry
            odo.update();

            // Get current position and heading
            Pose2D currentPosition = odo.getPosition();
            double currentX = currentPosition.getX(DistanceUnit.MM);
            double currentY = currentPosition.getY(DistanceUnit.MM);
            double currentHeading = currentPosition.getHeading(AngleUnit.DEGREES);

            // Calculate errors
            double errorX = targetX - currentX;
            double errorY = targetY - currentY;
            double headingError = targetHeading - currentHeading;

            // Normalize heading error to [-180, 180]
            headingError = (headingError + 180) % 360 - 180;

            // Calculate movement components
            double forwardPower = kP_Position * errorX; // Forward/backward movement
            double strafePower = kP_Position * errorY;  // Strafing movement
            double turnPower = kP_Heading * headingError;

            // Combine components into motor powers for mecanum drive
            double topLeftPower = forwardPower + strafePower - turnPower;
            double topRightPower = forwardPower - strafePower + turnPower;
            double bottomLeftPower = forwardPower - strafePower - turnPower;
            double bottomRightPower = forwardPower + strafePower + turnPower;

            // Clip motor powers to avoid overdriving
            topLeftPower = Math.max(-1.0, Math.min(1.0, topLeftPower));
            topRightPower = Math.max(-1.0, Math.min(1.0, topRightPower));
            bottomLeftPower = Math.max(-1.0, Math.min(1.0, bottomLeftPower));
            bottomRightPower = Math.max(-1.0, Math.min(1.0, bottomRightPower));

            // Set motor powers
            TopLeft.setPower(topLeftPower);
            TopRight.setPower(topRightPower);
            BottomLeft.setPower(bottomLeftPower);
            BottomRight.setPower(bottomRightPower);

            // Check if within tolerance
            if (Math.abs(errorX) < positionTolerance && Math.abs(errorY) < positionTolerance && Math.abs(headingError) < headingTolerance) {
                // Stop motors
                TopLeft.setPower(0);
                TopRight.setPower(0);
                BottomLeft.setPower(0);
                BottomRight.setPower(0);

                telemetry.addData("Status", "Target Reached");
                telemetry.update();
                break;
            }

            // Telemetry for debugging
            telemetry.addData("Target", "X: %.2f, Y: %.2f", targetX, targetY);
            telemetry.addData("Current", "X: %.2f, Y: %.2f, Heading: %.2f", currentX, currentY, currentHeading);
            telemetry.addData("Errors", "X: %.2f, Y: %.2f, Heading: %.2f", errorX, errorY, headingError);
            telemetry.update();
        }

        // Final stop to ensure no movement after loop exits
        TopLeft.setPower(0);
        TopRight.setPower(0);
        BottomLeft.setPower(0);
        BottomRight.setPower(0);
    }
}
