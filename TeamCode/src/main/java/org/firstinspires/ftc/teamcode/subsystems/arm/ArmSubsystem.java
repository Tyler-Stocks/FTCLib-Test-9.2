package org.firstinspires.ftc.teamcode.subsystems.arm;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.*;
import static org.firstinspires.ftc.teamcode.constants.Constants.ArmConstants.*;
import static org.firstinspires.ftc.teamcode.subsystems.arm.core.ArmHomingState.*;
import static org.firstinspires.ftc.teamcode.subsystems.arm.core.ArmState.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.arm.core.ArmHomingState;
import org.firstinspires.ftc.teamcode.subsystems.arm.core.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.utility.MotorUtility;

/**
 * <h1>ArmSubsystem</h1>
 * <br>
 * <p>
 *     Subsystem to encapsulate the entire arm mechanism. This includes the following components:
 *     <ul>
 *         <li>The worm and extension motors</li>
 *         <li>The worm and extension limit switches</li>
 *         <li>The worm potentiometer</li>
 *         <li>The outtake servos</li>
 *     </ul>
 * </p>
 */
public final class ArmSubsystem extends SubsystemBase {
    private ArmHomingState armHomingState;
    private ArmState armState;

    public final Outtake outtake;

    private final DcMotorImplEx wormMotor, elevatorMotor;

	private final RevTouchSensor wormLimitSwitch, elevatorLimitSwitch;

    private final AnalogInput wormPotentiometer;

    private final Telemetry telemetry;

    private int wormTargetPosition, elevatorTargetPosition;
    private int wormPosition, elevatorPosition;

    private double leftOuttakeDoorPosition, rightOuttakeDoorPosition;

    private boolean disabled;

    /**
     * Constructs, and initializes a new arm subsystem.
     * @param opMode The opmode you are running ; To obtain the hardwareMap and telemetry objects
     */
    public ArmSubsystem(@NonNull OpMode opMode) {
        disabled = false;

        telemetry = opMode.telemetry;

        HardwareMap hardwareMap = opMode.hardwareMap;

        outtake = new Outtake(hardwareMap);

        wormLimitSwitch     = hardwareMap.get(RevTouchSensor.class, WORM_LIMIT_SWITCH_NAME);
        elevatorLimitSwitch = hardwareMap.get(RevTouchSensor.class, ELEVATOR_LIMIT_SWITCH_NAME);

        wormMotor     = hardwareMap.get(DcMotorImplEx.class, WORM_MOTOR_NAME);
        elevatorMotor = hardwareMap.get(DcMotorImplEx.class, ELEVATOR_MOTOR_NAME);

	    DcMotorImplEx[] armMotors = new DcMotorImplEx[]{wormMotor, elevatorMotor};

        wormPotentiometer = hardwareMap.analogInput.get(WORM_POTENTIOMETER_NAME);

        wormMotor.setDirection(WORM_MOTOR_DIRECTION);
        wormMotor.setZeroPowerBehavior(WORM_MOTOR_ZERO_POWER_BEHAVIOR);
        wormMotor.setTargetPositionTolerance(WORM_MOTOR_POSITION_TOLERANCE_TICKS);

        elevatorMotor.setDirection(ELEVATOR_MOTOR_DIRECTION);
        elevatorMotor.setZeroPowerBehavior(ELEVATOR_MOTOR_ZERO_POWER_BEHAVIOR);
        elevatorMotor.setTargetPositionTolerance(ELEVATOR_MOTOR_POSITION_TOLERANCE_TICKS);

        MotorUtility.reset(armMotors);

        elevatorTargetPosition = 0;
        wormTargetPosition     = 0;

        elevatorPosition = 0;
        wormPosition     = 0;

        leftOuttakeDoorPosition  = 0.0;
        rightOuttakeDoorPosition = 0.0;

        armState       = ArmState.UNKNOWN;
        armHomingState = START;
    }

    @Override public void periodic() {
        if (disabled) return;

        wormPosition     = wormMotor.getCurrentPosition();
        elevatorPosition = elevatorMotor.getCurrentPosition();

        switch (armState) {
            case AT_POSITION:
                if (!wormMotor.isBusy()) wormMotor.setPower(0.0);
                break;
            case TO_POSITION:
                if (elevatorTargetPosition == 0 && wormTargetPosition == 0) {
                    if (wormPosition > WORM_SAFETY_LIMIT && elevatorPosition > 40) {
                        extendElevator(0);
                    } else if (wormPosition < WORM_SAFETY_LIMIT && elevatorPosition > 900) {
                        rotateWorm(WORM_SAFETY_LIMIT + 20);
                        extendElevator(1000);
                    } else if (90 < elevatorPosition && elevatorPosition < 900) {
                        extendElevator(0);
                    } else {
                        rotateWorm(wormTargetPosition);
                    }
                } else if (elevatorTargetPosition > 0 && wormPosition < WORM_SAFETY_LIMIT) {
                    wormTargetPosition = Math.max(wormTargetPosition, (WORM_SAFETY_LIMIT + 20));
                    rotateWorm(wormTargetPosition);
                } else {
                    rotateWorm(wormTargetPosition);

                    if (wormPosition > WORM_SAFETY_LIMIT) {
                        extendElevator(elevatorTargetPosition);
                    }
                }

                if (isAtPosition()) armState = AT_POSITION;
                break;
            case UNKNOWN:
                setHoming();
                break;
            case HOMING:
                home();
                break;
        }

        moveOuttake();
    }

    private void moveOuttake() {
        if (elevatorInFrame() && !isAtHome()) {
            outtake.closeOuttake();
        } else {
            outtake.setLeftOuttakeDoorPosition(leftOuttakeDoorPosition);
            outtake.setRightOuttakeDoorPosition(rightOuttakeDoorPosition);
        }
    }

    private void home() {
        switch (armHomingState) {
            case START:
                armState = HOMING;

                if (wormPotentiometer.getVoltage() >= WORM_SAFETY_VOLTAGE) {
                    wormMotor.setVelocity(0, DEGREES);
                    armHomingState = HOMING_ELEVATOR;
                } else {
                    wormMotor.setVelocity(MAX_WORM_VELOCITY_DEGREES_PER_SECOND, DEGREES);
                    elevatorMotor.setVelocity(percentOfMaxElevatorVelocity(-0.05), DEGREES);
                }

                // If the elevator limit switch and the worm limit switch is pressed, we know
                // that they are both home already so we can skip straight to removing the worm
                // backlash
                if (elevatorLimitSwitch.isPressed() && wormLimitSwitch.isPressed()) {
                    armHomingState = REMOVING_WORM_BACKLASH;
                }

                // If the elevator is homed, but the worm is not homed we can skipping homing the
                // elevator and move straight to homing the worm
                if (elevatorLimitSwitch.isPressed() && !wormLimitSwitch.isPressed()) {
                    armHomingState = HOMING_WORM;
                }

                break;
            case HOMING_ELEVATOR:
                elevatorMotor.setVelocity(ELEVATOR_HOMING_VELOCITY_DEGREES_PER_SECOND, DEGREES);

                if (elevatorLimitSwitch.isPressed()) {
                    armHomingState = HOMING_WORM;
                    MotorUtility.reset(elevatorMotor);
                }
                break;
            case HOMING_WORM:
                wormMotor.setVelocity(WORM_HOMING_VELOCITY_DEGREES_PER_SECOND, DEGREES);

                if (wormLimitSwitch.isPressed()) {
                    armHomingState = REMOVING_WORM_BACKLASH;
                    wormMotor.setVelocity(0);

                    // Helps smooth out the homing slightly
                    try { Thread.sleep(100); } catch (InterruptedException ignored) {}
                }
                break;
            case REMOVING_WORM_BACKLASH:
                wormMotor.setVelocity(WORM_BACKLASH_REMOVING_VELOCITY_DEGREES_PER_SECOND, DEGREES);

                if (!wormLimitSwitch.isPressed()) {
                    armHomingState = COMPLETE;
                    MotorUtility.reset(wormMotor);
                    rotateWorm(-10, 0.0);
                }
                break;
            case COMPLETE:
                MotorUtility.reset(wormMotor, elevatorMotor);
                armState       = AT_POSITION;
                armHomingState = IDLE;
                break;
            case IDLE:
                break;
        }
    }

    /**
     * Disables the arm subsystem
     */
    public void disable() {
        disabled = true;
    }

    /**
     * Tells the arm to home on the next iteration of update
     */
    public void setHoming() {
        armHomingState = START;
        armState       = HOMING;
    }

    /**
     * <p>
     *     Sets the target position of the worm and elevator motors. Note that these values will
     *     be clipped by the {@link Constants.ArmConstants#MIN_WORM_POSITION},
     *     {@link Constants.ArmConstants#MAX_WORM_POSITION},
     *     {@link Constants.ArmConstants#MIN_ELEVATOR_POSITION}, and
     *     {@link Constants.ArmConstants#MAX_ELEVATOR_POSITION} values found in
     *     {@link Constants.ArmConstants}.
     * </p>
     * @param wormTargetPosition     New worm target position
     * @param elevatorTargetPosition New elevator target position
     */
    public void setTargetPos(int wormTargetPosition,  int elevatorTargetPosition) {
        armState = TO_POSITION;

        this.elevatorTargetPosition = elevatorTargetPosition;
        this.wormTargetPosition     = wormTargetPosition;
    }

    private boolean isAtPosition() {
        return wormMotor.getTargetPosition()         == wormTargetPosition
                && elevatorMotor.getTargetPosition() == elevatorTargetPosition
                && !elevatorMotor.isBusy()
                && !wormMotor.isBusy();
    }

    private void extendElevator(int targetPos, double power) {
        elevatorMotor.setTargetPosition(targetPos);
        elevatorMotor.setMode(RUN_TO_POSITION);
        elevatorMotor.setPower(power);
    }

    private void extendElevator(int targetPosition) {
        extendElevator(targetPosition, DEFAULT_ELEVATOR_POWER);
    }

    private void rotateWorm(int targetPosition, double power) {
        wormMotor.setTargetPosition(targetPosition);
        wormMotor.setMode(RUN_TO_POSITION);
        wormMotor.setPower(power);
    }

    private void rotateWorm(int targetPosition) {
        rotateWorm(targetPosition, DEFAULT_WORM_POWER);
    }

    /**
     * @return Whether the arm is currently at home (0,0). Not accurate until homing is complete
     */
    public boolean isAtHome() {
        return elevatorTargetPosition == 0 && wormTargetPosition == 0 && armState == AT_POSITION;
    }

    /**
     * @return Whether the elevator is in frame. Not accurate until homing is complete
     */
    public boolean elevatorInFrame() {
        return elevatorPosition <= ELEVATOR_FRAME_LIMIT;
    }

    /**
     * @return The current position of the motor
     */
    public int wormPosition() {
        return wormMotor.getCurrentPosition();
    }

    /**
     * @return The target position of the motor reported by wormMotor.getTargetPosition()
     */
    public int wormTargetPosition() {
        return wormMotor.getTargetPosition();
    }

    /**
     * @return The local target position of the worm motor obtained through the local variable
     * wormMotorTargetPosition
     */
    public int localWormTargetPosition() {
        return wormTargetPosition;
    }

    /**
     * @return The current position of the elevator motor
     */
    public int elevatorPosition() {
        return elevatorMotor.getCurrentPosition();
    }

    /**
     * @return The target position of the elevator as reported by elevatorMotor.getTargetPosition()
     */
    public int elevatorTargetPosition() {
        return elevatorMotor.getTargetPosition();
    }

    /**
     * @return The local target position of the elevator motor obtained through the local variable
     * elevatorTargetPosition
     */
    public int localElevatorTargetPosition() {
        return elevatorTargetPosition;
    }

    /**
     * Displays debug information about the arm
     */
    public void debug() {
       telemetry.addLine("----- Arm Debug -----");
       telemetry.addData("Worm Current (AMPS)", wormMotor.getCurrent(AMPS));
       telemetry.addData("Elevator Current (AMPS)", elevatorMotor.getCurrent(AMPS));
       telemetry.addData("Worm Limit Switch Pressed", wormLimitSwitch.isPressed());
       telemetry.addData("Elevator Limit Switch Pressed", elevatorLimitSwitch.isPressed());
       telemetry.addData("Worm Position", wormMotor.getCurrentPosition());
       telemetry.addData("Elevator Position", elevatorMotor.getCurrentPosition());
       telemetry.addData("Worm Position (Local)", wormPosition);
       telemetry.addData("Elevator Position (Local)", elevatorPosition);
       telemetry.addData("Worm Target Position", wormMotor.getTargetPosition());
       telemetry.addData("Elevator Target Position", elevatorMotor.getTargetPosition());
       telemetry.addData("Worm Target Position (Local)", wormTargetPosition);
       telemetry.addData("Elevator Target Position (Local)", elevatorTargetPosition);
       telemetry.addData("Worm Direction", wormMotor.getDirection());
       telemetry.addData("Elevator Direction", elevatorMotor.getDirection());
       telemetry.addData("Potentiometer Voltage", wormPotentiometer.getVoltage());
       telemetry.addData("Arm State", armState);
       telemetry.addData("Arm Homing State", armHomingState);
       telemetry.addData("Worm Velocity", wormMotor.getVelocity(DEGREES));
       telemetry.addData("Is At Home", isAtHome());
    }
}
