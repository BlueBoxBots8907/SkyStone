package org.firstinspires.ftc.teamcode.autos.Configuration01;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.hardware.CollectionAndLiftMotors;
import org.firstinspires.ftc.teamcode.hardware.Servos;


@Autonomous(group = "autos")
public class Configuration01Left extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        Servos hooks = new Servos(hardwareMap);
        CollectionAndLiftMotors collectionMotors = new CollectionAndLiftMotors(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;
        //sampling here
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(-20, 10, 2.9))
                        .build()
        );
        //turn on the collection motors to sample
        collectionMotors.setCollectionPowers(0.6, 0.6);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(29)
                        .build()
        );
        sleep(500);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(20)
                        .build()
        );

        collectionMotors.setCollectionPowers(0, 0);
        drive.turnSync(Math.toRadians(112));
        //drive to the foundation
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(59)
                        .build()
        );

        //go back to get second block (while spitting out the first block)
        collectionMotors.setCollectionPowers(-0.6, -0.6);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(56)
                        .build()
        );
        collectionMotors.setCollectionPowers(0,0);
        drive.turnSync(Math.toRadians(-45));
        //get the second block
        collectionMotors.setCollectionPowers(0.6, 0.6);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(15)
                        .build()
        );
        sleep(750);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(15)
                        .build()
        );
        collectionMotors.setCollectionPowers(0,0);
        drive.turnSync(Math.toRadians(45));
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(65)
                        .build()
        );
        //score and park
        collectionMotors.setCollectionPowers(-0.6,-0.6);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(25)
                        .build()
        );
        collectionMotors.setCollectionPowers(0,0);
    }
}