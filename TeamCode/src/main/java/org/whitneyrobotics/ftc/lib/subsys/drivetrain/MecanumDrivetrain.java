package org.whitneyrobotics.ftc.lib.subsys.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

public interface MecanumDrivetrain {

    void operateWithOrientation(double leftPower, double rightPower);

    void operateWithOrientationScaled(double leftPower, double rightPower);

    void operate(double leftPower, double rightPower);

    void operateLeft(double leftPower);

    void operateRight(double rightPower);

    void switchOrientation(boolean gamepadInput);

    String getOrientation();

    double[] getEncoderDelta();

    double encToMM(double encoderTicks);

    void setRunMode(DcMotor.RunMode runMode);

}
