import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.TouchSensor;
import lejos.nxt.UltrasonicSensor;
import lejos.robotics.navigation.Pilot;
import lejos.robotics.navigation.TachoPilot;

public class Nick {
	public static void main(String[] args) {
		Nick nick = new Nick();
		
		nick.deliberate();
    }
	
	protected Pilot _pilot;
	
	protected TouchSensor _leftBumper;
	protected TouchSensor _rightBumper;
	
	protected UltrasonicSensor _headSensor;
	
	protected Motor _leftMotor;
	protected Motor _rightMotor;
	protected Motor _headMotor;
	
	public Nick() {
		this._leftBumper = new TouchSensor(SensorPort.S4);
		this._rightBumper = new TouchSensor(SensorPort.S1);
		
		this._headSensor = new UltrasonicSensor(SensorPort.S3);
		
		this._leftMotor = Motor.C;
		this._rightMotor = Motor.A;
		this._headMotor = Motor.B;
		
		this._headMotor.resetTachoCount();
		
		/** second parameter is approximate to the track width */
		this._pilot = new TachoPilot(2.2f, 8.66f, this._leftMotor, this._rightMotor);
	}
	
	public void deliberate() {
		while(!Button.ESCAPE.isPressed()) {
			if (this._isCollisionLeft() || this._isCollisionRight()) {
				this._log("Stop");
				this._pilot.stop();

				int collision_side = 0;
				
				if (this._isCollisionLeft()) {
					collision_side = 2;
				} else if (this._isCollisionRight()) {
					collision_side = 3;
				}
				
				this._log("Reverse");
				this._pilot.travel(-2);
				
				if (collision_side == 2) {
					this._log("Rotating right");
					this._pilot.rotate(-15);
				} else if (collision_side == 3) {
					this._log("Rotating left");
					this._pilot.rotate(15);
				}
				
//				this._log("Looking");
//				this._moveHead();
			} else {
				this._log("Forward");
				this._pilot.forward();
			}
		}
	}
	
	protected void _moveHead() {
		this._headMotor.rotate(75);
		this._headMotor.rotate(-(75*2));
		this._headMotor.rotate(75);
	}
	
	protected boolean _isCollisionLeft() {
		return this._leftBumper.isPressed() && this._log("Collision left");
	}
	
	protected boolean _isCollisionRight() {
		return this._rightBumper.isPressed() && this._log("Collision right");
	}
	
	protected boolean _log(String message) {
		LCD.clear();
		LCD.drawString("ARTHUR SAYS:", 0, 0);
		LCD.drawString(message, 0, 1);
		
		return true;
	}
}
