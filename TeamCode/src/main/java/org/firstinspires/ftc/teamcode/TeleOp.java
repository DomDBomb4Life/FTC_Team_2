
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.lang.Math;

public class ControllerModeV2 extends LinearOpMode {

    public static class DriveWheels {
        private DcMotor FrontL, FrontR, BackL, BackR;
        private LinearOpMode opMode;

        private final double MIN_POWER = 0.25;
        private final double DEFAULT_POWER = 0.5;
        private final double MAX_POWER = 1.0;


        double encoderStepsPerTileLinear = 1150;

        double encoderStepsPerTileStrafe = 3200;

        double encoderStepsPer90Deg = -850;
        double autonMotorPower = 0.5;

        public DriveWheels(LinearOpMode opMode) {
            this.opMode = opMode;

            FrontL = opMode.hardwareMap.get(DcMotor.class, "FrontL");
            BackL = opMode.hardwareMap.get(DcMotor.class, "BackL");
            FrontR = opMode.hardwareMap.get(DcMotor.class, "FrontR");
            BackR = opMode.hardwareMap.get(DcMotor.class, "BackR");

            FrontR.setDirection(DcMotorSimple.Direction.REVERSE);
            BackR.setDirection(DcMotorSimple.Direction.REVERSE);

            FrontR.setModec(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontL.setMode(DMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            FrontR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FrontL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }

        // Method to control the drive based on gamepad input
        public void drive() {
            double leftStickY = -opMode.gamepad1.left_stick_y;
            double leftStickX = opMode.gamepad1.left_stick_x;
            double pivot = opMode.gamepad1.right_stick_x;

            if (opMode.gamepad1.left_bumper){
                leftStickY = opMode.gamepad1.left_stick_y;
                leftStickX = -opMode.gamepad1.left_stick_x;
                pivot = -opMode.gamepad1.right_stick_x;
            }

            double POWER = DEFAULT_POWER;
            if (opMode.gamepad1.right_trigger > 0) {
                POWER = MAX_POWER;
            } else if (opMode.gamepad1.left_trigger > 0) {
                POWER = MIN_POWER;
            }


            // Set motor powers
            double frontRightPower = constrain(-pivot + (leftStickY - leftStickX), -1.0, 1.0) * POWER;
            double backRightPower = constrain(-pivot + leftStickY + leftStickX, -1.0, 1.0) * POWER;
            double frontLeftPower = constrain(pivot + leftStickY + leftStickX, -1.0, 1.0) * POWER;
            double backLeftPower = constrain(pivot + (leftStickY - leftStickX), -1.0, 1.0) * POWER;

            // Set motor powers
            FrontL.setPower(frontLeftPower);
            FrontR.setPower(frontRightPower);
            BackL.setPower(backLeftPower);
            BackR.setPower(backRightPower);
            // Telemetry for debugging

            opMode.telemetry.addData("FrontL", FrontL.getCurrentPosition());
            opMode.telemetry.addData("FrontR", FrontR.getCurrentPosition());
            opMode.telemetry.addData("BackL", BackL.getCurrentPosition());
            opMode.telemetry.addData("BackR", BackR.getCurrentPosition());
        }

        // Constraining method
        private double constrain(double var, double min, double max) {
            var = Math.min(Math.max(var, min), max);
            return var;
        }

        public void moveLinear(double tiles) {
            int encoderSteps = (int) (tiles * encoderStepsPerTileLinear);
            FrontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontL.setPower(autonMotorPower);
            FrontR.setPower(autonMotorPower);
            BackL.setPower(autonMotorPower);
            BackR.setPower(autonMotorPower);
            FrontR.setTargetPosition(encoderSteps);
            FrontL.setTargetPosition(encoderSteps);
            BackR.setTargetPosition(encoderSteps);
            BackL.setTargetPosition(encoderSteps);
            FrontR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public void turn90(int direction) {
            int encoderSteps = (int) (direction * encoderStepsPer90Deg);
            FrontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontL.setPower(autonMotorPower);
            FrontR.setPower(autonMotorPower);
            BackL.setPower(autonMotorPower);
            BackR.setPower(autonMotorPower);
            FrontR.setTargetPosition(encoderSteps);
            FrontL.setTargetPosition(-encoderSteps);
            BackR.setTargetPosition(encoderSteps);
            BackL.setTargetPosition(-encoderSteps);
            FrontR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public void waitForDrive() {
            while (BackL.isBusy() || BackR.isBusy() || FrontR.isBusy() || FrontL.isBusy()) {
                opMode.telemetry.addData("FrontL", FrontL.getCurrentPosition());
                opMode.telemetry.addData("FrontR", FrontR.getCurrentPosition());
                opMode.telemetry.addData("BackL", BackL.getCurrentPosition());
                opMode.telemetry.addData("BackR", BackR.getCurrentPosition());
                opMode.telemetry.update();
            }

        }
    }
    public void waitForDrive()
        {
        while (BackL.isBusy() || BackR.isBusy() || FrontR.isBusy() || FrontL.isBusy()) {
            opMode.telemetry.addData("FrontL", FrontL.getCurrentPosition());
            opMode.telemetry.addData("FrontR", FrontR.getCurrentPosition());
            opMode.telemetry.addData("BackL", BackL.getCurrentPosition());
            opMode.telemetry.addData("BackR", BackR.getCurrentPosition());
            opMode.telemetry.update();
        }

    }

    public class TeleOp {
    
}
