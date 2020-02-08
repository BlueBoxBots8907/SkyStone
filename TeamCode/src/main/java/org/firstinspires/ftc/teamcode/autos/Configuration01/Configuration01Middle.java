package org.firstinspires.ftc.teamcode.autos.Configuration01;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.hardware.CollectionAndLiftMotors;
import org.firstinspires.ftc.teamcode.hardware.Servos;

@Autonomous(group = "autos")
public class Configuration01Middle extends LinearOpMode {
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
                        .back(15)
                        .build()
        );
        drive.turnSync(Math.toRadians(135));
        //turn on the collection motors to sample
        collectionMotors.setCollectionPowers(0.6, 0.6);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(34)
                        .build()
        );
        sleep(1000);
        collectionMotors.setCollectionPowers(0, 0);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(15)
                        .build()
        );
        drive.turnSync(Math.toRadians(135));
        //drive to the foundation
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(57)
                        .build()
        );
        //go back to get second block (while spitting out the first block)
        collectionMotors.setCollectionPowers(-0.6, -0.6);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(62)
                        .build()
        );
        collectionMotors.setCollectionPowers(0,0);
        drive.turnSync(Math.toRadians(-45));
        //get the second block
        collectionMotors.setCollectionPowers(0.6, 0.6);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(12)
                        .build()
        );
        sleep(750);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(12)
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
                        .back(17)
                        .build()
        );
        collectionMotors.setCollectionPowers(0,0);
    }
}