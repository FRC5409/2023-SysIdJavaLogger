package frc.robot;

public class Constants {

    public static final int DATA_VECTOR_SIZE = 36000;

    public static final class kMotor {
        public static final int id_leftFrontDrive           = 6;
        public static final int id_leftRearDrive            = 14;

        public static final int id_rightFrontDrive          = 4;
        public static final int id_rightRearDrive           = 15;

        public static final boolean leftInverted            = false;
        public static final boolean rightInverted           = true;

        public static final double gearing                  = 1.0;

        public static final double distancePerRotation      = 0.1 * Math.PI;
    }

    public static final class kCANCoder {
        public static final int id_leftEncoder              = 11;
        public static final int id_rightEncoder             = 10;
        public final static double enc_CountsPerRevolution  = 4096;
        public final static double enc_SensorCoefficient    = (Math.PI * kWheel.wheelDiameter) / enc_CountsPerRevolution;
        public final static String enc_UnitString           = "m";

        public static final boolean leftInverted            = false;
        public static final boolean rightInverted           = true;
    }

    public static class kWheel {
        public final static double wheelDiameter            = 0.1; // metres, placeholder value
        public final static double wheelCircumference       = Math.PI * wheelDiameter; // metres
    }

    public static final class kGyro {
        public static final int id_gyro                     = 23;
    }
}
