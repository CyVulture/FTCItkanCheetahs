package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="test code")
public class LeftRightTest extends LinearOpMode {
    DcMotor bottomRight;
    DcMotor bottomLeft;
    DcMotor topRight;
    DcMotor topLeft;

    @Override
    public void runOpMode() throws InterruptedException {
        bottomRight = hardwareMap.get(DcMotor.class, "Bottom Right");
        bottomLeft = hardwareMap.get(DcMotor.class, "Bottom Left");
        topRight = hardwareMap.get(DcMotor.class, "Top Right");
        topLeft = hardwareMap.get(DcMotor.class, "Top Left");
        double motorPower = 0.3;

        bottomRight.setPower(motorPower);
        bottomLeft.setPower(motorPower);
        topRight.setPower(-motorPower);
        topLeft.setPower(-motorPower);
        sleep(1000);
    }


}
