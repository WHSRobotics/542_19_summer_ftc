package org.whitneyrobotics.ftc.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    DcMotor rightLift;
    DcMotor leftLift;
    public Lift(HardwareMap liftMap) {
        rightLift = liftMap.dcMotor.get("rightLift");
        leftLift = liftMap.dcMotor.get("leftLift");
    }

}
