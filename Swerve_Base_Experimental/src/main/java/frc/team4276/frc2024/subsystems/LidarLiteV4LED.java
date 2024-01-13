package frc.team4276.frc2024.subsystems;

import java.util.TimerTask;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Timer;
import frc.team4276.frc2024.Robot;

public class LidarLiteV4LED {
	private I2C i2c;
	private byte[] distance;
	private java.util.Timer updater;
	private LIDARUpdater task;

	private final int LIDAR_ADDR = 0x62;
	private final int LIDAR_CONFIG_REGISTER = 0x00;
	private final int LIDAR_DISTANCE_REGISTER = 0x10; // Read 2 bytes from this address and you get the high byte too
	private final int LIDAR_CMD_TAKE_MEASUREMENT = 0x04;

	public LidarLiteV4LED(Port port) {
		i2c = new I2C(port, LIDAR_ADDR);

		distance = new byte[2];

		task = new LIDARUpdater();
		updater = new java.util.Timer();
	}

	// Distance in cm
	public int getDistance() {
		return (int) Integer.toUnsignedLong(distance[1] << 8) + Byte.toUnsignedInt(distance[0]);
	}

	public double pidGet() {
		return getDistance();
	}

	// Start 10Hz polling
	public void start() {
		updater.scheduleAtFixedRate(task, 0, 1000);
	}

	// Start polling for period in milliseconds
	public void start(int period) {
		updater.scheduleAtFixedRate(task, 0, period);
	}

	public void stop() {
		updater.cancel();
	}

	// Update distance variable
	public void update() {

		i2c.write(LIDAR_CONFIG_REGISTER, LIDAR_CMD_TAKE_MEASUREMENT); // Initiate measurement
		Timer.delay(0.006); // Delay for measurement to be taken

		boolean isFailed = i2c.read(LIDAR_DISTANCE_REGISTER, 2, distance); // Read in measurement
		Robot.g_isValidLidar = !(isFailed);

		if (getDistance() > 0) {
			Robot.g_lidarDistanceCentimeters = getDistance();
		} else {
			Robot.g_isValidLidar = false;
		}
	}

	// Timer task to keep distance updated
	private class LIDARUpdater extends TimerTask {
		public void run() {
			while (true) {
				update();

				try {
					Thread.sleep(10);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		}
	}
}