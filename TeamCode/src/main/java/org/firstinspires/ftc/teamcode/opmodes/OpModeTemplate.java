package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS;
import static org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit.VOLTS;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.*;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.hanger.HangerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.launcher.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ledcontroller.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.mosaicfixers.MosaicFixerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.odometry.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.purplepixelplacer.PurplePixelPlacerSubsystem;

public abstract class OpModeTemplate extends CommandOpMode {
	protected enum GamePeriod {
		TELEOP,
		ENDGAME
	}

	private LynxModule[] lynxModules;

	protected GamePeriod gamePeriod = GamePeriod.TELEOP;

	protected DriveSubsystem driveSubsystem;
	protected HangerSubsystem hangerSubsystem;
	protected IntakeSubsystem intakeSubsystem;
	protected ArmSubsystem armSubsystem;
	protected MosaicFixerSubsystem mosaicFixerSubsystem;
	protected LEDSubsystem ledSubsystem;
	protected PurplePixelPlacerSubsystem purplePixelPlacerSubsystem;
	protected LauncherSubsystem launcherSubsystem;
	protected OdometrySubsystem odometrySubsystem;
	protected OuttakeSubsystem outtakeSubsystem;

	protected GamepadEx driverGamepad, operatorGamepad;

	/**
	 * Configures all of the button bindings for the opmode
	 */
    protected void configureBindings() {

    }

	/**
	 * Configures all of the triggers that are completely independent of button presses. Some
	 * triggers that are coupled with bindings are declared in configureBindings().
	 */
	protected void configureTriggers() {

	}

	/**
	 * <p>Initializes the hardware in the following order:</p>
	 * <ol>
	 *     <lI>Checks the battery voltage. If it is below 10 volts, throws an error.</lI>
	 *     <li>Sets the lynx modules to bulk caching mode.</li>
	 *     <lI>Initializes the gamepads.</lI>
	 *     <li>Sets the subsystem default commands.</li>
	 *     <lI>Registers the subsystems with the command scheduler.</lI>
	 *     <li>Initializes the over-current protection command</li>
	 * </ol>
	 */
	protected final void initializeHardware() {
		configureLynxModules();
		checkBatteryVoltage();

		driveSubsystem             = new DriveSubsystem(this);
		hangerSubsystem            = new HangerSubsystem(this);
		intakeSubsystem            = new IntakeSubsystem(this);
		armSubsystem               = new ArmSubsystem(this);
		mosaicFixerSubsystem       = new MosaicFixerSubsystem(this);
		ledSubsystem               = new LEDSubsystem(this);
		purplePixelPlacerSubsystem = new PurplePixelPlacerSubsystem(this);
		launcherSubsystem          = new LauncherSubsystem(this);
		odometrySubsystem          = new OdometrySubsystem(this);
		outtakeSubsystem           = new OuttakeSubsystem(this);

		initializeGamepads();

		register(
				driveSubsystem,
				hangerSubsystem,
				intakeSubsystem,
				armSubsystem,
				mosaicFixerSubsystem,
				ledSubsystem,
				purplePixelPlacerSubsystem,
				launcherSubsystem,
				odometrySubsystem,
				outtakeSubsystem
		);

		schedule(new RunCommand(this::terminateOnOverCurrent));
	}

	/**
	 * Initializes the gamepads. Note that this function must be called before configureBindings()
	 * unless you have previously called initializeHardware().
	 */
	protected final void initializeGamepads() {
		operatorGamepad = new GamepadEx(gamepad2);
		driverGamepad   = new GamepadEx(gamepad1);
	}

	/**
	 * @return Whether gamePeriod == GamePeriod.ENDGAME
	 */
	protected final boolean isEndgame() {
		return gamePeriod == GamePeriod.ENDGAME;
	}

	/**
	 * @return Whether gamePeriod = GamePeriod.TELEOP
	 */
	protected final boolean isTeleOp() {
		return gamePeriod == GamePeriod.TELEOP;
	}

	/**
	 * Forces the telemetry to stop running in the background by setting the transmission interval
	 * to {@link Integer#MAX_VALUE}.
	 */
	protected final void forceStopTelemetry() {
		telemetry.setMsTransmissionInterval(Integer.MAX_VALUE);
	}

	private void configureLynxModules() {
		lynxModules = hardwareMap.getAll(LynxModule.class).toArray(new LynxModule[0]);

		for (LynxModule lynxModule: lynxModules) {
			lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
		}
	}

	private void checkBatteryVoltage() {
		if (lynxModules == null) return;

		for (LynxModule lynxModule : lynxModules) {
			double voltage = lynxModule.getInputVoltage(VOLTS);

			if (voltage < 10.0) throw new BatteryVoltageError(voltage, this);
		}
	}

	private void terminateOnOverCurrent() {
		if (lynxModules == null) return;

		double totalCurrentDraw = 0;

		for (LynxModule lynxModule : lynxModules) {
			totalCurrentDraw += lynxModule.getCurrent(AMPS);
		}

		if (totalCurrentDraw > 18.0) throw new OverCurrentError(totalCurrentDraw, this);
	}

	protected final void debug() {
		telemetry.addData("Current Period", gamePeriod);
	}

	protected final static class ToggleOpModeCommand extends CommandBase {
		private final OpModeTemplate opMode;

		public ToggleOpModeCommand(@NonNull OpModeTemplate opMode) {
			this.opMode = opMode;
		}

		@Override public void execute() {
			switch (opMode.gamePeriod) {
				case TELEOP:
					opMode.gamePeriod = GamePeriod.ENDGAME;
					break;
				case ENDGAME:
					opMode.gamePeriod = GamePeriod.TELEOP;
					break;
			}
		}

		@Override public boolean isFinished() {
			return true;
		}
	}

	protected final static class EndgameTrigger extends Trigger {
		private final OpModeTemplate opMode;

		public EndgameTrigger(@NonNull OpModeTemplate opMode) {
			this.opMode = opMode;
		}

		@Override public boolean get() {
			return this.opMode.isEndgame();
		}
	}

	protected final static class TeleOpTrigger extends Trigger {
		private final OpModeTemplate opMode;

		public TeleOpTrigger(@NonNull OpModeTemplate opMode) {
			this.opMode = opMode;
		}

		@Override public boolean get() {
			return this.opMode.isTeleOp();
		}
	}

	private static class BatteryVoltageError extends Error {
		public BatteryVoltageError(double voltage, @NonNull OpModeTemplate opMode) {
			super("Batter Voltage Is Below Safe Level Of " + voltage + ".");

			opMode.terminateOpModeNow();
		}
	}

	private static class OverCurrentError extends Error {
		public OverCurrentError(double currentAmps, @NonNull OpModeTemplate opMode) {
			super("Current Is Above Safe Level Of 18 Amps\n"
					+ "Measured Current: " + currentAmps + " Amps");

			opMode.terminateOpModeNow();
		}
	}
}
