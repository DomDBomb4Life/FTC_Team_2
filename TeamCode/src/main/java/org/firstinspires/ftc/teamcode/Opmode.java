package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Drive")
public class Opmode extends LinearOpMode {

    public static class DriveWheels {
        DcMotor FrontL, BackL, FrontR, BackR;

        public DriveWheels(LinearOpMode opMode) {
            FrontL = opMode.hardwareMap.get(DcMotor.class, "FrontL");
            BackL = opMode.hardwareMap.get(DcMotor.class, "BackL");
            FrontR = opMode.hardwareMap.get(DcMotor.class, "FrontR");
            BackR = opMode.hardwareMap.get(DcMotor.class, "BackR");

            BackL.setDirection(DcMotorSimple.Direction.REVERSE);
            FrontL.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public void setMotorPowers(double fl, double bl, double fr, double br) {
            FrontL.setPower(fl);
            BackL.setPower(bl);
            FrontR.setPower(fr);
            BackR.setPower(br);
        }
    }

    public class Arm {
        //declares motors and servos
        private ServoMotor liftL, liftR, arm;
        private Servo wrist;
        private OpMode opmode;


        public Arm(OpMode opmode) {
            //init OpMode
            this.opmode = opmode;
            //init motors
            liftL = opmode.hardwareMap.get(ServoMotor.class, "LiftL");
            liftR = opmode.hardwareMap.get(ServoMotor.class, "LiftR");
            liftR.setDirection(ServoMotorSimple.Direction.REVERSE);
            arm = opmode.hardwareMap.get(ServoMotor.class, "Arm");
            liftR.setMode(ServoMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftL.setMode(ServoMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(ServoMotor.RunMode.STOP_AND_RESET_ENCODER);
            //init the wrist
            wrist = opmode.hardwareMap.get(Servo.class, "Wrist");
        }
    }

    @Override
    public void runOpMode() {
        DriveWheels driveWheels = new DriveWheels(this);
        waitForStart();
        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y;  // Negative because the joystick is inverted
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            // Calculate motor powers
            double frontLeftPower = drive + strafe + turn;
            double backLeftPower = drive - strafe + turn;
            double frontRightPower = drive - strafe - turn;
            double backRightPower = drive + strafe - turn;

            // Normalize the values so that no wheel power exceeds 1.0
            double max = Math.max(Math.abs(frontLeftPower),
                    Math.max(Math.abs(backLeftPower),
                            Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))));

            if (max > 1.0) {
                frontLeftPower /= max;
                backLeftPower /= max;
                frontRightPower /= max;
                backRightPower /= max;
            }

            // Set motor powers
            driveWheels.setMotorPowers(frontLeftPower, backLeftPower, frontRightPower, backRightPower);

            // Add telemetry for debugging
            telemetry.addData("Front Left Power", frontLeftPower);
            telemetry.addData("Back Left Power", backLeftPower);
            telemetry.addData("Front Right Power", frontRightPower);
            telemetry.addData("Back Right Power", backRightPower);
            telemetry.update();
        }
    }
}
