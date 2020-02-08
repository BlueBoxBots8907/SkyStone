package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.Arrays;
import java.util.List;

public class CollectionAndLiftMotors {
    private ExpansionHubMotor CollectionLeft, CollectionRight, LiftTop, LiftBottom;

    public CollectionAndLiftMotors(HardwareMap hardwareMap) {
        CollectionLeft = hardwareMap.get(ExpansionHubMotor.class, "CollectionLeft");
        CollectionRight = hardwareMap.get(ExpansionHubMotor.class, "CollectionRight");
        LiftTop = hardwareMap.get(ExpansionHubMotor.class, "LiftTop");
        LiftBottom = hardwareMap.get(ExpansionHubMotor.class, "LiftBottom");

        CollectionLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        CollectionRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        LiftTop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        LiftBottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        CollectionLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        LiftBottom.setDirection(DcMotorSimple.Direction.REVERSE);

//        LiftTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        LiftBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setCollectionPowers(double m1, double m2) {
        CollectionLeft.setPower(m1);
        CollectionRight.setPower(m2);
    }

    public void setLiftPowers(double p1, double p2) {
        LiftTop.setPower(p1);
        LiftBottom.setPower(p2);
    }

    public double getLiftHeight() {
        return LiftBottom.getCurrentPosition() / 103.6 * Math.PI * 36.0 / 25.4;
    }
}

