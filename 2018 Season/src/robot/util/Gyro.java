package robot.util;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class Gyro extends ADXRS450_Gyro {

	public Gyro() {
		super();
	}

	@Override
	public double getAngle() {
		return super.getAngle() % 360;
	}

	public double getSplitAngle() {
		if (getAngle() > 180) {
			return getAngle() - 360;
		} else if (getAngle() < -180) {
			return getAngle() + 360;
		} else {
			return getAngle();
		}
	}

}
