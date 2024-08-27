package org.firstinspires.ftc.teamcode.subsystems.utility;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoUtility {

    /**
     * Sets the position of the input servos
     * @param position The position to set each input servo
     * @param servos The servos to set the position of
     */
    public static void setPositions(double position, @NonNull Servo ... servos) {
        for (Servo servo : servos) {
            servo.setPosition(position);
        }
    }
}
