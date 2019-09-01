package lib.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class    RobotConstants {
    public static double DEADBAND_DRIVE_TO_TARGET = 24.5;
    public static double DEADBAND_ROTATE_TO_TARGET = 1.0;
    public static double drive_min = .2;//.1245;
    public static double drive_max = 1.0;//.6;
    public static double rotate_min = 0.2;
    public static double rotate_max = 1.0;
    public static double R_KP = 1.542;//1.19;
    public static double R_KI = 0.3;
    public static double R_KD = 0.36;
    public static double D_KP = 1.7;//1.5;
    public static double D_KI = 0.7;
    public static double D_KD = 0.8;

    public static double S_KP = 0.00007;
    public static double S_KV = 0.00077;
    public static double S_KA = 0.0085;

    public static double S_VC = 2;
}
