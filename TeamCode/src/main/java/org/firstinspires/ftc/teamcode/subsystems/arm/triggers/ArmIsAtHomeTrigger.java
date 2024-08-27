package org.firstinspires.ftc.teamcode.subsystems.arm.triggers;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;

/**
 * <h1ArmIsAtHomeTrigger</h1>
 * <br>
 * <p>
 *     The ArmIsAtHomeTrigger returns true when the position of the worm and elevator motors are 0
 *     and neither of the aforementioned motors are busy.
 * </p>
 */
public class ArmIsAtHomeTrigger extends Trigger {
    private final ArmSubsystem armSubsystem;

    /**
     * Constructs a new ArmIsAtHomeTrigger
     * @param armSubsystem The arm subsystem to read from
     */
    public ArmIsAtHomeTrigger(@NonNull ArmSubsystem armSubsystem) {
       this.armSubsystem = armSubsystem;
    }

    @Override public boolean get() {
        return armSubsystem.isAtHome();
    }
}
