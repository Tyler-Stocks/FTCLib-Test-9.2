package org.firstinspires.ftc.teamcode.constants;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.opencv.core.Core;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

public final class Constants {

    public static class FileConstants {
        @SuppressLint("sdCardPath")
        public static final String CONSTANTS_FILE_LOCATION
            = "/sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode/Constants/";
    }

    public static class LauncherConstants {
        public static String LAUNCHER_SERVO_NAME = "launcherServo";

        public static double LAUNCH_POSITION = 0.6;
        public static double ZERO_POSITION   = 0.0;
    }

    public static class HangerConstants {
        public static String HANGER_SERVO_NAME = "hangerServo";

        public static double HANG_POSITION = 1.0;
        public static double ZERO_POSITION = 0.0;
    }

    public static class IntakeConstants {
        public static String INTAKE_MOTOR_NAME     = "intakeMotor";
        public static String FRONT_BEAM_BREAK_NAME = "frontBeamBreak";
        public static String BACK_BEAM_BREAK_NAME  = "backBeamBreak";

        public static double INTAKE_POWER              = 1.0;
        public static double OUTTAKE_POWER             = 0.4;
        public static double INTAKE_TRIGGER_THRESHOLD  = 0.2;
        public static double OUTTAKE_TRIGGER_THRESHOLD = 0.2;
    }

    public static class ArmConstants {
        public static String WORM_MOTOR_NAME            = "wormMotor";
        public static String ELEVATOR_MOTOR_NAME        = "elevatorMotor";
        public static String WORM_LIMIT_SWITCH_NAME     = "wormLimitSwitch";
        public static String ELEVATOR_LIMIT_SWITCH_NAME = "elevatorLimitSwitch";
        public static String WORM_POTENTIOMETER_NAME    = "wormPotentiometer";

        public static DcMotorSimple.Direction ELEVATOR_MOTOR_DIRECTION
                = DcMotorSimple.Direction.REVERSE;
        public static DcMotorSimple.Direction WORM_MOTOR_DIRECTION
                = DcMotorSimple.Direction.FORWARD;

        public static DcMotor.ZeroPowerBehavior ELEVATOR_MOTOR_ZERO_POWER_BEHAVIOR
                = DcMotor.ZeroPowerBehavior.BRAKE;
        public static DcMotor.ZeroPowerBehavior WORM_MOTOR_ZERO_POWER_BEHAVIOR
                = DcMotor.ZeroPowerBehavior.BRAKE;

        public static double DEFAULT_WORM_POWER     = 1.0;
        public static double DEFAULT_ELEVATOR_POWER = 1.0;

        public static double MAX_WORM_VELOCITY_DEGREES_PER_SECOND = 270;
        public static double MAX_ELEVATOR_VELOCITY_DEGREES_PER_SECOND = 280;

        /**
         * @param percent The percent. For example 0.5 or 0.01. If you go over 1 the motor simply
         *                won't go any faster
         * @return The input percentage of the max worm velocity in degrees per second. For example
         * setting percent to 0.05 would return MAX_WORM_VELOCITY_DEGREES_PER_SECOND * 0.05
         */
        public static double percentageOfMaxWormVelocity(double percent) {
           return MAX_WORM_VELOCITY_DEGREES_PER_SECOND * percent;
        }

        /**
         * @param percent The percent. For example 0.5 or 0.01. If you go over 1 the motor simply
         *                won't go any faster.
         * @return The input percentage of the max elevator velocity in degrees per second. For
         * example setting percent to 0.05 would return
         * MAX_ELEVATOR_VELOCITY_DEGREES_PER_SECOND * 0.05.
         */
        public static double percentOfMaxElevatorVelocity(double percent) {
            return MAX_ELEVATOR_VELOCITY_DEGREES_PER_SECOND * percent;
        }

        public static double WORM_BACKLASH_REMOVING_VELOCITY_DEGREES_PER_SECOND
                = percentageOfMaxWormVelocity(0.25);

        public static double WORM_HOMING_VELOCITY_DEGREES_PER_SECOND
                = percentageOfMaxWormVelocity(-0.7);

        public static double ELEVATOR_HOMING_VELOCITY_DEGREES_PER_SECOND
                = percentOfMaxElevatorVelocity(-0.8);

        public static double WORM_SAFETY_VOLTAGE = 0.8;

        public static int WORM_SAFETY_LIMIT     = 700;
        public static int MAX_ELEVATOR_POSITION = 2750;
        public static int MAX_WORM_POSITION     = 2500;
        public static int MIN_ELEVATOR_POSITION = -100;
        public static int MIN_WORM_POSITION     = -500;

        public static int BOTTOM_ELEVATOR_POSITION  = 1580;
        public static int BOTTOM_WORM_POSITION      = 1250;
        public static int LOW_ELEVATOR_POSITION     = 1782;
        public static int LOW_WORM_POSITION         = 1432;
        public static int MEDIUM_ELEVATOR_POSITION  = 2107;
        public static int MEDIUM_WORM_POSITION      = 1713;
        public static int HIGH_ELEVATOR_POSITION    = 2415;
        public static int HIGH_WORM_POSITION        = 1822;
        public static int TOP_ELEVATOR_POSITION     = 2667;
        public static int TOP_WORM_POSITION         = 1909;

        public static int ELEVATOR_MOTOR_POSITION_TOLERANCE_TICKS = 30;
        public static int WORM_MOTOR_POSITION_TOLERANCE_TICKS     = 20;

        public static int ELEVATOR_FRAME_LIMIT = 950;

        // -----------------------------------------------------------------------------------------
        // Outtake Constants
        // -----------------------------------------------------------------------------------------

        public static String LEFT_OUTTAKE_SERVO_NAME  = "leftOuttakeServo";
        public static String RIGHT_OUTTAKE_SERVO_NAME = "rightOuttakeServo";

        public static double OPEN_POSITION = 0.25;
        public static double CLOSED_POSITION = 0.0;
    }

    public static class DrivebaseConstants {
       public static String FRONT_LEFT_DRIVE_MOTOR_NAME  = "frontLeftDriveMotor";
       public static String FRONT_RIGHT_DRIVE_MOTOR_NAME = "frontRightDriveMotor";
       public static String BACK_LEFT_DRIVE_MOTOR_NAME   = "backLeftDriveMotor";
       public static String BACK_RIGHT_DRIVE_MOTOR_NAME  = "backRightDriveMotor";

       public static double DRIVE_DEAD_ZONE  = 0.05;
       public static double STRAFE_DEAD_ZONE = 0.05;
       public static double TURN_DEAD_ZONE   = 0.05;
    }

    public static class PurplePixelPlacerConstants {
        public static String PURPLE_PIXEL_PLACER_SERVO_NAME = "purplePixelPlacerServo";

        public static Servo.Direction PURPLE_PIXEL_PLACER_SERVO_DIRECTION = Servo.Direction.FORWARD;

        public static double PURPLE_PIXEL_PLACER_NEUTRAL_POSITION = 1.0;
        public static double PURPLE_PIXEL_PLACER_ACTIVE_POSITION  = 0.0;
    }

    public static class MosaicFixerConstants {
        public static String LEFT_MOSAIC_FIXER_SERVO_NAME  = "leftMosaicFixerServo";
        public static String RIGHT_MOSAIC_FIXER_SERVO_NAME = "rightMosaicFixerServo";

        public static Servo.Direction LEFT_MOSAIC_FIXER_SERVO_DIRECTION  = Servo.Direction.REVERSE;
        public static Servo.Direction RIGHT_MOSAIC_FIXER_SERVO_DIRECTION = Servo.Direction.FORWARD;

        public static double LEFT_MOSAIC_FIXER_RETRACTED_POSITION = 0.04;
        public static double LEFT_MOSAIC_FIXER_LOW_POSITION       = 0.62;
        public static double LEFT_MOSAIC_FIXER_MEDIUM_POSITION    = 0.59;
        public static double LEFT_MOSAIC_FIXER_HIGH_POSITION      = 0.54;

        public static double RIGHT_MOSAIC_FIXER_RETRACTED_POSITION = 0.0;
        public static double RIGHT_MOSAIC_FIXER_MEDIUM_POSITION = 0.61;
        public static double RIGHT_MOSAIC_FIXER_HIGH_POSITION = 0.55;
    }

    public static class AprilTagConstants {
        public static float DECIMATION = 3;

        public static int EXPOSURE_MS               = 20;
        public static int GAIN                      = 2;
        public static int WHITE_BALANCE_TEMPERATURE = 4000;

        public static double FX = 660.75;
        public static double FY = 660.75;
        public static double CX = 323.034;
        public static double CY = 230.681;
    }

    public static class PropDetectionConstants {
        public static double LEFT_X  = 0.25;
        public static double RIGHT_X = 0.75;

        public static int ERODE_PASSES = 9;

        public static Scalar LOW_HSV_RANGE_BLUE  = new Scalar(97, 100, 0);
        public static Scalar HIGH_HSV_RANGE_BLUE = new Scalar(125, 255, 255);

        public static Scalar LOW_HSV_RANGE_RED_ONE  = new Scalar(160, 100, 0);
        public static Scalar HIGH_HSV_RANGE_RED_ONE = new Scalar(180, 255, 255);

        public static Scalar LOW_HSV_RANGE_RED_TWO  = new Scalar(0, 100, 0);
        public static Scalar HIGH_HSV_RANGE_RED_TWO = new Scalar(10, 255, 255);

        public static final Point CV_ANCHOR        = new Point(-1, -1);
        public static Scalar CV_BORDER_VALUE = new Scalar(-1);
        public static int CV_BORDER_TYPE           = Core.BORDER_CONSTANT;
    }

    public static class OdometryConstants {
        public static String ODOMETRY_NAME = "odometry";

        public static SparkFunOTOS.Pose2D OFFSET = new SparkFunOTOS.Pose2D(0,0,0);

        public static int IMU_CALIBRATION_SAMPLES = 255;

        public static double ANGULAR_SCALAR = 1.02;
        public static double LINEAR_SCALAR  = 1.0;

        /**
         * Boolean flag to determine whether the imu calibration runs asynchronously. If true,
         * the imu calibration function will block until it is finished, or fails. If false, the
         * imu calibration function will being calibration, and return early letting the calibration
         * continue in the background.
         */
        public static boolean BLOCK_ON_IMU_CALIBRATION = true;
    }

    public static class LEDControllerConstants {
        public static String LEFT_LED_RED_CHANNEL_NAME    = "leftLEDRedChannel";
        public static String LEFT_LED_GREEN_CHANNEL_NAME  = "leftLEDGreenChannel";
        public static String RIGHT_LED_RED_CHANNEL_NAME   = "rightLEDRedChannel";
        public static String RIGHT_LED_GREEN_CHANNEL_NAME = "rightLEDGreenChannel";
    }

    public static class ServoTestConstants {
        public static String TEST_SERVO_NAME               = "TestServo";
        public static double TEST_SERVO_START_POSITION     = 0.00;
        public static Servo.Direction TEST_SERVO_DIRECTION = Servo.Direction.FORWARD;
    }
}
