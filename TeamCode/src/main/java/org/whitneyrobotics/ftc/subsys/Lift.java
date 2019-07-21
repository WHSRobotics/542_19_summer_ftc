package org.whitneyrobotics.ftc.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import lib.util.Toggler;

public class Lift {

    DcMotor rightLift;
    DcMotor leftLift;
    Toggler liftTog;

    private final double LIFT_POWER = 1.0;

    public Lift(HardwareMap liftMap) {
        rightLift = liftMap.dcMotor.get("rightLift");
        leftLift = liftMap.dcMotor.get("leftLift");
        liftTog = new Toggler(16);
    }
    private final int[] liftPositionArray = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

    public void operateLift(boolean gamepadInputUp, boolean gamepadInputDown, boolean gamepadInputReset){

        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setPower(LIFT_POWER);
        leftLift.setPower(LIFT_POWER);

        liftTog.changeState(gamepadInputUp, gamepadInputDown);
        if (gamepadInputReset) {
            liftTog.setState(0);
        }

        rightLift.setTargetPosition(liftPositionArray[liftTog.currentState()]);
        leftLift.setTargetPosition(liftPositionArray[liftTog.currentState()]);
    }

    public void setLiftPosition(int targetPosition){

        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setPower(LIFT_POWER);
        leftLift.setPower(LIFT_POWER);

        liftTog.setState(targetPosition);
        rightLift.setTargetPosition(liftPositionArray[liftTog.currentState()]);
        leftLift.setTargetPosition(liftPositionArray[liftTog.currentState()]);
    }

    public int getRightLiftEncoder(){
        return rightLift.getCurrentPosition();
    }

    public int getLeftLiftEncoder(){
        return leftLift.getCurrentPosition();
    }

}
