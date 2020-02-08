package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.CollectionAndLiftMotors;
import org.firstinspires.ftc.teamcode.hardware.Servos;

@TeleOp(group = "teleop")
public class deleteLater extends LinearOpMode {
    private ElapsedTime resetCapstoneTimer = new ElapsedTime();

    private DcMotor FLM, FRM, BLM, BRM;

    @Override
    public void runOpMode() throws InterruptedException {
        //Set controller values
        double LeftStickY;
        double LeftStickX;
        double RightStickX;
        double leftTrigger;
        double rightTrigger;
        double DriveSpeed = 1;
        Servos servos = new Servos(hardwareMap);
        CollectionAndLiftMotors collectionAndLift = new CollectionAndLiftMotors(hardwareMap);


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


        boolean clawOpen = true;

        float armPosition = 0;
        float liftEncoderValues = 0;
        float liftEncoderPrevious = 0;

        double collectionPower = 0;
        double oldLiftPower = 0;

        // Resetting code
        boolean isResetting = false;
        ElapsedTime resettingTime = new ElapsedTime();

//        servos.foundationUp();
        boolean capped = false;
        servos.initPower();
        servos.clawClose();
        servos.setArmPosition(0.28);
        waitForStart();
        resetCapstoneTimer.startTime();
        while (!isStopRequested()) {
            /*drive.setDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            ));*/
            //Read the gamepad
            LeftStickY = -(Math.pow(gamepad1.left_stick_y, 2) * Math.signum(gamepad1.left_stick_y) * DriveSpeed);
            LeftStickX = -(Math.pow(gamepad1.left_stick_x, 2) * Math.signum(gamepad1.left_stick_x) * DriveSpeed);
            RightStickX = (Math.pow(gamepad1.right_stick_x, 2)* Math.signum(gamepad1.right_stick_x) * DriveSpeed);
            leftTrigger = gamepad2.left_trigger;
            rightTrigger = gamepad2.right_trigger;
            //Add the gamepad movements into the telemetry
            telemetry.addData("leftStickY", LeftStickY);
            telemetry.addData("leftStickX", LeftStickX);
            telemetry.addData("rightStickX", RightStickX);
            telemetry.addData("leftTrigger", leftTrigger);
            telemetry.addData("rightTrigger", rightTrigger);
/*
            if (gamepad1.left_bumper) {
                DriveSpeed = 1;
                servos.foundationUp();
            } else if (gamepad1.right_bumper) {
                DriveSpeed = 0.35;
                servos.foundationDown();
            }
            else if (gamepad1.right_trigger >= 0.2)
                DriveSpeed = 0.5;

*/
            //Motor movement
            FLM.setPower((LeftStickY - LeftStickX) + RightStickX);
            BLM.setPower((LeftStickY + LeftStickX) + RightStickX);
            FRM.setPower(LeftStickY + LeftStickX - RightStickX);
            BRM.setPower((LeftStickY - LeftStickX) - RightStickX);

            /*placeholder lift movement
            if (gamepad2.dpad_up || gamepad2.dpad_down) {
                if (gamepad2.dpad_down && liftEncoderValues > 100) {
                    liftEncoderValues -= 400;
                }
                else if (gamepad2.dpad_up && liftEncoderValues < 3400)
                    liftEncoderPrevious = liftEncoderValues;
                    liftEncoderValues += 400;

                //TODO: set encoders to liftEncoderValues
                while (gamepad2.dpad_up || gamepad2.dpad_down){}
            }
            if (gamepad2.dpad_right) {
                liftEncoderValues = 0;
                //TODO: set encoders to liftEncoderValues
            }
            if (gamepad2.dpad_left) {
                liftEncoderValues = liftEncoderPrevious + 400;
            }
*/
/*
            //arm movement
            if (gamepad2.left_bumper) {
                servos.setArmPosition(0.28);
            } else if (gamepad2.right_bumper) {
                if (capped)
                    servos.setArmPosition(0.83);
                else
                    servos.setArmPosition(0.9);
            }
           /* if (openClawTimer.milliseconds() > 1000 && openClawTimer.milliseconds() < 1400) {
                servos.clawOpen();
            }*/
           /* if (gamepad2.y)
                servos.clawHalfOpen();
            else if (gamepad2.x)    
                servos.clawClose();

          /*  if (gamepad2.left_stick_button) {
                isResetting = true;
                resettingTime.reset();
            }

            if (isResetting) {
                if (resettingTime.seconds() <= 0.5) {
                    servos.clawDrop();
                } else if (resettingTime.seconds() <= 1) {
                    collectionAndLift.setLiftPowers(0.5, 0.5);
                } else if (collectionAndLift.getLiftHeight() > 4) {
                    servos.setArmPosition(0.36);
                    collectionAndLift.setLiftPowers(-0.3, -0.3);
                } else {
                    servos.clawOpen();
                    collectionAndLift.setLiftPowers(0, 0);
                    isResetting = false;
                }
            }*/
/*
            if (gamepad2.a) {
                resetCapstoneTimer.reset();
                capped = true;
            servos.dropCapstone();
        }
        if (resetCapstoneTimer.milliseconds() >= 1000 && resetCapstoneTimer.milliseconds() <= 1500)
            servos.liftCapstone();

        if(gamepad2.b)
            servos.liftCapstone();
            //claw movement
          /*  if (gamepad2.x) {
                if (clawOpen && cooldown.milliseconds() > 1000) {
                    cooldown.reset();
                    servos.clawClose();
                    clawOpen = false;
                }
                else if (cooldown.milliseconds() > 1000){
                    cooldown.reset();
                    servos.clawOpen();
                    clawOpen = true;
                }
            }
            if (gamepad2.y) {
                servos.clawStop();
            telemetry.addData("Stopped Servo at:", cooldown.milliseconds()); } */
            //collection movement
            double newCollectionPower = ((gamepad1.right_trigger - (gamepad1.left_trigger * 2.5     )) * .2);

            if (newCollectionPower != collectionPower) {
               servos.clawOpen();
                collectionPower = newCollectionPower;
                collectionAndLift.setCollectionPowers(collectionPower, collectionPower);
            }
/*
            //lift movement
            double liftPower = gamepad2.left_stick_y != 0 || collectionAndLift.getLiftHeight() > 1 ?
                    -Math.pow(gamepad2.left_stick_y, 3) * (gamepad2.left_stick_y > 0 ? 0.4 : 0.5) + .2 : // .25 = bias power (counteract gravity)
                    0;

            if (liftPower != oldLiftPower && !isResetting) {
                collectionAndLift.setLiftPowers(liftPower, liftPower);
                oldLiftPower = liftPower;
            }

//            telemetry.addData("Arm Pos.", armPosition);
//            telemetry.addData("Claw Opened", clawOpen);
//            telemetry.addData("New Lift Power", newCollectionPower);
//            telemetry.addData("Old Lift Power", collectionPower);
//            telemetry.addData("Gunner Left Trigger", gamepad2.left_trigger);
//            telemetry.update(); */
        }
    }
}

