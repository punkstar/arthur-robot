import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.TouchSensor;
import lejos.nxt.UltrasonicSensor;
import lejos.robotics.navigation.Pilot;
import lejos.robotics.navigation.TachoPilot;

import java.lang.InterruptedException;

public class Arthur {
	public static void main(String[] args) {
		Arthur arthur = new Arthur();
		
		arthur.deliberate();
    }
	
	protected static final int COLLISION_NONE = 0;
	protected static final int COLLISION_LEFT = 3;
	protected static final int COLLISION_RIGHT = 4;
	protected static final int COLLISION_BOTH = 5;
	
	protected static final int ROTATE_90 = 4800;
	
	protected Pilot _pilot;
	
	protected TouchSensor _leftBumper;
	protected TouchSensor _rightBumper;
	
	protected UltrasonicSensor _headSensor;
	
	protected Motor _leftMotor;
	protected Motor _rightMotor;
	protected Motor _headMotor;
	
	public Arthur() {
		this._leftBumper = new TouchSensor(SensorPort.S4);
		this._rightBumper = new TouchSensor(SensorPort.S1);
		
		this._headSensor = new UltrasonicSensor(SensorPort.S3);
		
		this._leftMotor = Motor.A;
		this._rightMotor = Motor.C;
		this._headMotor = Motor.B;
		
		this._headMotor.resetTachoCount();
		this._headMotor.setSpeed(1440);
		
		this._pilot = new TachoPilot(0.56f, 1.18f, this._leftMotor, this._rightMotor);
	}
	
	public void deliberate() {
		this._actStartup();
		
		while(!this._shouldQuit()) {
			if (this._isCollisionLeft() || this._isCollisionRight()) {
				this._pilot.stop();
				int collision = COLLISION_NONE;
				
				this._sleep(500);
				
				if (this._isCollisionBoth()) {
					this._log("Collision both");
					collision = COLLISION_BOTH;
					this._actCollisionBoth();
				} else if (this._isCollisionLeft()) {
					this._log("Collision left");
					collision = COLLISION_LEFT;
					this._actCollisionSingle(collision);
				} else if (this._isCollisionRight()) {
					this._log("Collision right");
					collision = COLLISION_RIGHT;
					this._actCollisionSingle(collision);
				} else {
					this._log("No collision?!");
					this._pilot.forward();
				}
			} else {
				this._log("Moving along");
			}
		}
		
		this._actFinish();
	}
	
	protected void _actStartup() {
		int closestDistance = 255;
		int closestTacho = 0;
		int measurement = 255;
		
		this._log("Starting up");
		
		while (this._headMotor.isMoving()) {
		}
		
		this._headMotor.rotate(ROTATE_90, true);
		while (this._headMotor.isMoving()) {
			measurement = this._headSensor.getDistance();
			if (measurement < closestDistance) {
				closestDistance = measurement;
				closestTacho = this._headMotor.getTachoCount();
			}
		}
		this._log("LS: "+closestDistance+" @ "+this._tachoToDegrees(closestTacho));
		this._headMotor.rotate(-ROTATE_90);
		this._headMotor.rotate(-ROTATE_90, true);
		while (this._headMotor.isMoving()) {
			measurement = this._headSensor.getDistance();
			if (measurement < closestDistance) {
				closestDistance = measurement;
				closestTacho = this._headMotor.getTachoCount();
			}
		}
		this._log("RS: "+closestDistance+" @ "+this._tachoToDegrees(closestTacho));
		this._headMotor.rotate(ROTATE_90);
		
		int angle = this._tachoToDegrees(closestTacho);
		this._log("Angle: "+angle);
		
		this._pilot.rotate(angle);
		this._pilot.forward();
	}
	
	protected void _actCollisionBoth() {
	}
	
	protected void _actCollisionSingle(int side) {
		int angle = 0;
		if (side == COLLISION_LEFT) {
			angle = -30;
			this._log("Colision left.");
		} else if (side == COLLISION_RIGHT) {
			angle = 30;
			this._log("Colision right.");
		} else {
			// FAIL
			this._log("OMGWTFBBQ!");
		}
		
		this._pilot.travel(-1); // in wheel rotations
		this._pilot.rotate(angle);
		
		this._pilot.forward();
	}
	
	protected void _actFinish() {
		this._pilot.stop();
		
		while (this._headMotor.isMoving()) {
		}
		
		int angle = this._headMotor.getTachoCount();
		angle = -angle;
		
		if (angle != 0) {
			this._log("Head reset: "+this._tachoToDegrees(angle));
			this._headMotor.rotate(angle);
		}
		
		this._log("Good night!");
		this._sleep(2000);
		
		System.exit(0);
	}
	
	protected boolean _isCollisionLeft() {
		return this._leftBumper.isPressed(); // || this._leftMotor.isStopped();
	}
	
	protected boolean _isCollisionRight() {
		return this._rightBumper.isPressed(); // || this._rightMotor.isStopped();
	}
	
	protected boolean _isCollisionBoth() {
		return this._isCollisionLeft() && this._isCollisionRight();
	}
	
	protected boolean _log(String message) {
		LCD.clear();
		LCD.drawString("ARTHUR SAYS:", 0, 0);
		LCD.drawString(message, 0, 1);
		
		return true;
	}
	
	protected boolean _shouldQuit() {
		if (Button.ESCAPE.isPressed()) {
			return true;
		}
		
		return false;
	}
	
	protected void _sleep(long ms) {
		try {
			Thread.sleep(ms);
		} catch (InterruptedException e) {}
	}
	
	protected int _tachoToDegrees(int tacho) {
		return 90 * tacho / ROTATE_90;
	}
}
