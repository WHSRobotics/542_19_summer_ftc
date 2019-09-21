package org.whitneyrobotics.ftc.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.whitneyrobotics.ftc.subsys.Lift;

public class liftEncoderPositionTest extends OpMode{
    Lift lift;

    @Override
    public void init() {
       lift = new Lift(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("Left Encoder Value", lift.getLeftLiftEncoder());
        telemetry.addData("Right Encoder Value", lift.getRightLiftEncoder());
    }
}
