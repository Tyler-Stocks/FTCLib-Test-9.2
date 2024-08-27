package org.firstinspires.ftc.teamcode.subsystems.launcher;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import static com.qualcomm.robotcore.hardware.Servo.Direction.*;

import static org.firstinspires.ftc.teamcode.constants.Constants.LauncherConstants.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * <h1>Launcher Subsystem</h1>
 * <br>
 * <p>
 *     Subsystem to encapsulate the servo which controls the launching of the airplane. Contains
 *     the following hardware
 *     <ul>
 *         <li>Launcher Servo</li>
 *     </ul>
 * </p>
 */
public class LauncherSubsystem extends SubsystemBase {
    private final Servo launcherServo;

    private final Telemetry telemetry;

    /**
     * Constructs a new LauncherSubsystem
     * @param opMode The opMode you are running ; To obtain the hardwareMap and telemetry
     */
    public LauncherSubsystem(@NonNull OpMode opMode) {
        telemetry = opMode.telemetry;

        launcherServo = opMode.hardwareMap.get(Servo.class, LAUNCHER_SERVO_NAME);

        launcherServo.setDirection(REVERSE);

    }

    /**
     * Sets the launcher servo to the launch position
     */
    public void launch() {
        launcherServo.setPosition(LAUNCH_POSITION);
    }

    /**
     * Sets the launcher servo to the zero position
     */
    public void reset() { launcherServo.setPosition(ZERO_POSITION); }

    /**
     * Displays debug information about the launcher
     */
    public void debug() {
        telemetry.addLine("---- Launcher Debug ----");
        telemetry.addData("Servo Position", launcherServo.getPosition());
        telemetry.addData("Servo Direction", launcherServo.getDirection());
    }
}
