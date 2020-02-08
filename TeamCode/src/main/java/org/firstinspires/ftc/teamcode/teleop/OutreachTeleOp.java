package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(group = "teleop")
public class OutreachTeleOp extends LinearOpMode {

    private DcMotor FLM, FRM, BLM, BRM;


    @Override
    public void runOpMode() throws InterruptedException {
        //Set controller values
        double LeftStickY;
        double LeftStickX;
        double RightStickX;
        double DriveSpeed = 1;


        FLM = hardwareMap.get(DcMotor.class, "leftFront");
        FRM = hardwareMap.get(DcMotor.class, "rightFront");
        BLM = hardwareMap.get(DcMotor.class, "leftRear");
        BRM = hardwareMap.get(DcMotor.class, "rightRear");



        FLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FLM.setDirection(DcMotorSimple.Direction.REVERSE);
        BLM.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        while (!isStopRequested()) {
            LeftStickY = -(gamepad1.left_stick_y * DriveSpeed);
            LeftStickX = -(gamepad1.left_stick_x * DriveSpeed);
            RightStickX = (gamepad1.right_stick_x * DriveSpeed);
            //Add the gamepad movements into the telemetry
            telemetry.addData("leftStickY", LeftStickY);
            telemetry.addData("leftStickX", LeftStickX);
            telemetry.addData("rightStickX", RightStickX);

            if (gamepad1.left_bumper)
                DriveSpeed = 1;
            else if (gamepad1.right_bumper)
                DriveSpeed = 0.25;

            //Motor movement
            FLM.setPower((LeftStickY - LeftStickX) + RightStickX);
            BLM.setPower((LeftStickY + LeftStickX) + RightStickX);
            FRM.setPower(LeftStickY + LeftStickX - RightStickX);
            BRM.setPower((LeftStickY - LeftStickX) - RightStickX);

            telemetry.update();
        }
    }
}

