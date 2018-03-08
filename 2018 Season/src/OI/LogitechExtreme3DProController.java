package OI;

public class LogitechExtreme3DProController extends GameController{

	public LogitechExtreme3DProController(int port) {
		super(port);
	}

	public double getXAxis(){
		return joystick.getRawAxis(0);
	}
	
	public double getYAxis(){
		return joystick.getRawAxis(1);
	}
	
	public double getRotateAxis(){
		return joystick.getRawAxis(2);
	}
	
	public double getSlider(){
		return joystick.getRawAxis(3);
	}

	public boolean isButtonOne(){
		return joystick.getRawButton(0);
	}
	
	public boolean isButtonTwo(){
		return joystick.getRawButton(1);
	}
	
	public boolean isButtonThree(){
		return joystick.getRawButton(2);
	}
	
	public boolean isButtonFour(){
		return joystick.getRawButton(3);
	}
	
	public boolean isButtonFive(){
		return joystick.getRawButton(4);
	}
	
	public boolean isButtonSix(){
		return joystick.getRawButton(5);
	}
	
	public boolean isButtonSeven(){
		return joystick.getRawButton(6);
	}
	
	public boolean isButtonEight(){
		return joystick.getRawButton(7);
	}
	
	public boolean isButtonNine(){
		return joystick.getRawButton(8);
	}
	
	public boolean isButtonTen(){
		return joystick.getRawButton(9);
	}
	
	public boolean isButtonEleven(){
		return joystick.getRawButton(10);
	}
	
	public boolean isButtonTwelve(){
		return joystick.getRawButton(11);
	}

	@Override
	public boolean isADown() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean isBDown() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean isXDown() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean isYDown() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean isLeftBumperDown() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean isRightBumperDown() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean isButton1Down() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean isButton2Down() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean isLeftThumbDown() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean isRightThumbDown() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public double getLeftXAxis() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double getLeftYAxis() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double getLeftTriggerAxis() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double getRightTriggerAxis() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double getRightXAxis() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double getRightYAxis() {
		// TODO Auto-generated method stub
		return 0;
	}
	
}
