package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.hardware.CollectionAndLiftMotors;
import org.firstinspires.ftc.teamcode.hardware.Servos;

@TeleOp(group = "teleop")
public class AutoBlockDepositTest extends LinearOpMode {
    private ElapsedTime cooldown = new ElapsedTime();

    private DcMotor FLM, FRM, BLM,BRM;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap);
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



        boolean clawOpen = true;

        float armPosition = 0;
        float liftEncoderValues = 0;
        float liftEncoderPrevious = 0;

        double collectionPower = 0;

//        servos.foundationUp();
        servos.initPower();
        waitForStart();
        servos.clawOpen();
        servos.setArmPosition(1);
        sleep(1000);
        collectionAndLift.setCollectionPowers(0.57, 0.57);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(8)
                        .build()
        );
        collectionAndLift.setCollectionPowers(0,0);
        servos.setArmPosition(1);
        sleep(500);
        servos.clawClose();
        collectionAndLift.setCollectionPowers(0.6, 0.6);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(7)
                        .build()
        );
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(15)
                        .build()
        );
        collectionAndLift.setCollectionPowers(0,0);
        sleep(1000);
        servos.setArmPosition(0.5);
        sleep(1000);
        servos.clawOpen();
        sleep(500);
        servos.setArmPosition(1);
        sleep(2500);
    }
}

