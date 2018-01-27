package OI;

public class XBoxController extends GameController {

	public XBoxController(int port) {
		super(port);
	}

	@Override
	public boolean isADown() {
		return joystick.getRawButton(1);
	}

	@Override
	public boolean isBDown() {
		return joystick.getRawButton(2);
	}

	@Override
	public boolean isXDown() {
		return joystick.getRawButton(3);
	}

	@Override
	public boolean isYDown() {
		return joystick.getRawButton(4);
	}

	@Override
	public boolean isLeftBumperDown() {
		return joystick.getRawButton(5);
	}

	@Override
	public boolean isRightBumperDown() {
		return joystick.getRawButton(6);
	}

	@Override
	public boolean isButton1Down() {
		return joystick.getRawButton(7);
	}

	@Override
	public boolean isButton2Down() {
		return joystick.getRawButton(8);
	}

	@Override
	public boolean isLeftThumbDown() {
		return joystick.getRawButton(9);
	}

	@Override
	public boolean isRightThumbDown() {
		return joystick.getRawButton(10);
	}

	@Override
	public double getLeftXAxis() {
		return joystick.getRawAxis(0);
	}

	@Override
	public double getLeftYAxis() {
		return joystick.getRawAxis(1);
	}

	@Override
	public double getLeftTriggerAxis() {
		return joystick.getRawAxis(2);
	}

	@Override
	public double getRightTriggerAxis() {
		return joystick.getRawAxis(3);
	}

	@Override
	public double getRightXAxis() {
		return joystick.getRawAxis(4);
	}

	@Override
	public double getRightYAxis() {
		return joystick.getRawAxis(5);
	}

}
