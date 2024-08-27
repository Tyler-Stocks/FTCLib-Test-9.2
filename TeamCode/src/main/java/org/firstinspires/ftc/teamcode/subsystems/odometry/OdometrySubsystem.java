package org.firstinspires.ftc.teamcode.subsystems.odometry;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.teamcode.constants.Constants.OdometryConstants.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public final class OdometrySubsystem extends SubsystemBase {
	private final Telemetry telemetry;
	private final SparkFunOTOS odometry;

	public OdometrySubsystem(@NonNull OpMode opMode) {
		telemetry = opMode.telemetry;

		odometry = opMode.hardwareMap.get(SparkFunOTOS.class, ODOMETRY_NAME);

		configureOdometry();
	}

	public void reset() {
		odometry.resetTracking();
	}

	public void setPosition(@NonNull SparkFunOTOS.Pose2D pose) {
		odometry.setPosition(pose);
	}

	public SparkFunOTOS.Pose2D acceleration() {
		return odometry.getAcceleration();
	}

	public SparkFunOTOS.Pose2D position() {
		return odometry.getPosition();
	}

	public double heading() {
		return odometry.getPosition().h;
	}

	public double y() {
		return odometry.getPosition().y;
	}

	public double x() {
		return odometry.getPosition().x;
	}

	public boolean isConnected() {
		return odometry.isConnected();
	}

	private void configureOdometry() {
		if (!isConnected()) throw new OdometryNotConnectedError();

		odometry.setAngularUnit(DEGREES);
		odometry.setLinearUnit(INCH);

		odometry.setOffset(OFFSET);

		odometry.setAngularScalar(ANGULAR_SCALAR);
		odometry.setLinearScalar(LINEAR_SCALAR);

		calibrateIMU();

		reset();

		if (!odometry.begin()) throw new OdometryFailedToBeginError();
	}

	/**
	 * Calibrates the IMU. If the IMU cannot be calibrated
	 */
	public void calibrateIMU() {
		if (odometry.calibrateImu(IMU_CALIBRATION_SAMPLES, BLOCK_ON_IMU_CALIBRATION)) {
			telemetry.addLine("IMU Calibration Successful");
		} else {
			throw new IMUCalibrationError();
		}

		telemetry.update();
	}

	private static class IMUCalibrationError extends Error {
		public IMUCalibrationError() {
			super("Failed To Calibrate SparkFunOTOS Imu.");
		}
	}

	private static class OdometryNotConnectedError extends Error {
		public OdometryNotConnectedError() {
			super("Sparkfun OTOS IMU not connected to I2C port. Check wiring and configuration.");
		}
	}

	private static class OdometryFailedToBeginError extends Error {
		public OdometryFailedToBeginError() {
			super("Sparkfun OTOS failed to start. Check wiring and configuration");
		}
	}
}
