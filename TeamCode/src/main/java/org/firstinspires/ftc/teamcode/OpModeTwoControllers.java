package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="OpModeTwoControllers")
public class OpModeTwoControllers extends LinearOpMode {

    private DcMotor topRightMotor;
    private DcMotor topLeftMotor;
    private DcMotor bottomRightMotor;
    private DcMotor bottomLeftMotor;
    private DcMotor armMotor;
    private Servo wrist;
    private Servo intakeWheel;
    private boolean intakeWheelOn;
    private DcMotor armExtensionMotor;
    @Override
    public void runOpMode() {

        telemetry.addData("test", "test");
        topRightMotor = hardwareMap.get(DcMotor.class, "Top Right");
        topLeftMotor = hardwareMap.get(DcMotor.class, "Top Left");
        bottomRightMotor = hardwareMap.get(DcMotor.class, "Bottom Right");
        bottomLeftMotor = hardwareMap.get(DcMotor.class, "Bottom Left");
        armMotor = hardwareMap.get(DcMotor.class, "Arm Motor");
        wrist = hardwareMap.get(Servo.class, "Wrist");
        intakeWheel = hardwareMap.get(Servo.class, "Intake Wheel");
        armExtensionMotor = hardwareMap.get(DcMotor.class, "Arm Extension Motor");

        // reverse left motors so they move in the same direction
        topLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        bottomLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // these variables make it so that the robot doesn't go too fast
        double armPower = 0.5;
        double wheelPowerFactor = 0.5;


        waitForStart();

        while (opModeIsActive()) {
            // movement of robot
            double y = gamepad1.left_stick_y; // forward/backward
            double x = gamepad1.left_stick_x;  // left/right
            double rotation = -gamepad1.right_stick_x;  // rotation

            double topLeftPower = y + x + rotation;
            double topRightPower = y - x - rotation;
            double bottomLeftPower = y - x + rotation;
            double bottomRightPower = y + x - rotation;
            double armMotorPower;
            boolean keepItConstant = true;

            topLeftPower = topLeftPower * wheelPowerFactor;
            topRightPower = topRightPower * wheelPowerFactor;
            bottomLeftPower = bottomLeftPower * wheelPowerFactor;
            bottomRightPower = bottomRightPower * wheelPowerFactor;


            topLeftMotor.setPower(topLeftPower);
            topRightMotor.setPower(topRightPower);
            bottomLeftMotor.setPower(bottomLeftPower);
            bottomRightMotor.setPower(bottomRightPower);

            boolean intakeWheelOn = false;

            // wrist movement
            // double pos = wrist.getPosition();
            // double offset = 0.005;
            // if (gamepad2.dpad_left) {
            //     wrist.setPosition(pos+offset);
            // } else if (gamepad2.dpad_right) {
            //     wrist.setPosition(pos-offset);
            // }

            // wrist movement
            double pos = wrist.getPosition();
            double offset = 0.005;
            if (gamepad2.dpad_left) {
                wrist.setPosition(0.95);
            } else if (gamepad2.dpad_right) {
                wrist.setPosition(0.6);
            }

            // intake wheel
            if (gamepad2.x){
                intakeWheel.setPosition(1.0);
            }
            else if (gamepad2.b){
                intakeWheel.setPosition(0.0);
            }
            else if (gamepad2.a){
                intakeWheel.setPosition(0.5);
            }

            //intake wheel
            // if (gamepad1.x){
            //             intakeWheel.setPosition(1.0);
            //         }
            // else if (gamepad1.b){
            //         intakeWheel.setPosition(0.0);
            //     }
            // else if (gamepad1.a){
            //         intakeWheel.setPosition(0.5);
            //     }

            //arm motor movement
            // the robot will use this value to keep the arm from falling down too fast
            double keepArmConstantPower = 0.3;
            // boolean isStrong = false;\
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (gamepad2.dpad_up == true){
                armMotor.setPower(0.8);
            }
            else if (gamepad2.dpad_down == true){
                armMotor.setPower(-0.8);
            }
            else if (gamepad2.y){
                armMotor.setPower(-5);
            }
            // // for ascent ; pressing y will make the robot's arm strength stronger or weaker
            // if ((gamepad1.y) && !(isStrong)){
            //     armPower=2*armPower;
            //     isStrong = true;
            // }
            // else if ((gamepad1.y) && (isStrong)){
            //     armPower=armPower/2;
            //     isStrong = false;
            // }
            // // keep the arm constant by constantly changing its power to an equilibrium withou
            // else {
            //     armMotor.setPower(keepArmConstantPower);
            //     sleep(5);
            //     armMotor.setPower(-keepArmConstantPower);
            // }
            else{
                armMotor.setPower(0);
            }
            //linear slide extension code
            if (gamepad1.dpad_left) {
                armExtensionMotor.setPower(0.5);
            } else if (gamepad1.dpad_right) {
                armExtensionMotor.setPower(-0.5);
            }
            else{
                armExtensionMotor.setPower(0);
            }
            //telemetry

            double armPos = armMotor.getCurrentPosition();
            double wristPosition = wrist.getPosition();
            double intakeWheelPosition = intakeWheel.getPosition();
            telemetry.addData("Top Left Power", topLeftPower);
            telemetry.addData("Top Right Power", topRightPower);
            telemetry.addData("Bottom Left Power", bottomLeftPower);
            telemetry.addData("Bottom Right Power", bottomRightPower);
            telemetry.addData("Wrist Position", wristPosition);
            telemetry.addData("Intake Wheel Position", intakeWheel.getPosition());
            telemetry.addData("Arm Power", armMotor.getPower());
            telemetry.addData("Keep it Constant", keepItConstant);
            // telemetry.addData("Arm Position", armMotor.getPosition());
            // telemetry.addData("Is the arm at full power?", isStrong);

            telemetry.update();
        }



    }


    public void move(double motorPower){

    }
}


