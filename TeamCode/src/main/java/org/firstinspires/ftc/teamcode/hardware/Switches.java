package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.lang.reflect.Array;

public class Switches {
    private DigitalChannel
            SwitchAllianceBlue,
            SwitchAllianceRed,
            SwitchParkingBridge,
            SwitchParkingWall,
            SwitchFoundationUs,
            SwitchFoundationPartner,
            SwitchStartLoading,
            SwitchStartBuilding;
    boolean SABState; boolean SARState;
    boolean SSLState; boolean SSBState;
    boolean SFUState; boolean SFPState;
    boolean SPBState; boolean SPWState;
    public Switches(HardwareMap hardwareMap) {

        SwitchAllianceBlue = hardwareMap.get(DigitalChannel.class, "SwitchAllianceBlue");
        SwitchAllianceRed = hardwareMap.get(DigitalChannel.class, "SwitchAllianceRed");
        SwitchStartLoading = hardwareMap.get(DigitalChannel.class, "SwitchStartLoading");
        SwitchStartBuilding = hardwareMap.get(DigitalChannel.class, "SwitchStartBuilding");
        SwitchFoundationUs = hardwareMap.get(DigitalChannel.class, "SwitchFoundationUs");
        SwitchFoundationPartner = hardwareMap.get(DigitalChannel.class, "SwitchFoundationPartner");
        SwitchParkingBridge = hardwareMap.get(DigitalChannel.class, "SwitchParkingBridge");
        SwitchParkingWall = hardwareMap.get(DigitalChannel.class, "SwitchParkingWall");

        SwitchAllianceBlue.setMode(DigitalChannel.Mode.INPUT);
        SwitchStartLoading.setMode(DigitalChannel.Mode.INPUT);
        SwitchFoundationPartner.setMode(DigitalChannel.Mode.INPUT);
        SwitchParkingBridge.setMode(DigitalChannel.Mode.INPUT);
    }
    public boolean[] readSwitches(){
        SwitchAllianceBlue.setMode(DigitalChannel.Mode.INPUT);
        SwitchAllianceRed.setMode(DigitalChannel.Mode.INPUT);
        SwitchStartLoading.setMode(DigitalChannel.Mode.INPUT);
        SwitchFoundationPartner.setMode(DigitalChannel.Mode.INPUT);
        SwitchParkingBridge.setMode(DigitalChannel.Mode.INPUT);
        SABState = SwitchAllianceBlue.getState(); SARState = SwitchAllianceRed.getState();
        SSLState = SwitchStartLoading.getState(); SSBState = SwitchStartBuilding.getState();
        SFUState = SwitchFoundationUs.getState(); SFPState = SwitchFoundationPartner.getState();
        SPBState = SwitchParkingBridge.getState(); SPWState = SwitchParkingWall.getState();
        boolean[] switchStates = {SABState, SSLState, SFUState, SPBState, SARState, SSBState, SFPState, SPWState};
        return switchStates;
    }
}
