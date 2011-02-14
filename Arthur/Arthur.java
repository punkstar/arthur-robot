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
		while(!Button.ESCAPE.isPressed()) {
			if (this._isCollisionLeft() || this._isCollisionRight()) {
				this._log("Stop");
				this._pilot.stop();
				
				this._sleep(500);

				int collision = 0;
				
				if (this._isCollisionBoth()) {
					this._log("Collision both");
					collision = COLLISION_BOTH;
				} else if (this._isCollisionLeft()) {
					this._log("Collision left");
					collision = COLLISION_LEFT;
				} else if (this._isCollisionRight()) {
					this._log("Collision right");
					collision = COLLISION_RIGHT;
				}
				
				this._sleep(500);
				
				this._log("Reversing..");
				this._pilot.travel(-1);
				
				this._sleep(500);
				
				switch (collision) {
					case COLLISION_BOTH:
						this._handleHeadOnCollision();
						break;
					case COLLISION_LEFT:
						this._log("Rotating right");
						this._pilot.rotate(-30);
						break;
					case COLLISION_RIGHT:
					default:
						this._log("Rotating left");
						this._pilot.rotate(30);
						break;
				}
				
				
			} else {
				this._log("Forward..");
				this._pilot.forward();
			}
		}
	}
	
	protected void _handleHeadOnCollision() {
		int data = 255;
		this._headMotor.rotate(ROTATE_90, true);
		while (this._headMotor.isMoving()) {
			this._headSensor.ping();
			data = this._headSensor.getDistance();
			this._log("Dist: "+data);
		}
		this._headMotor.rotate(-ROTATE_90*2);
		this._headMotor.rotate(ROTATE_90);
	}
	
	protected void _moveHead() {
		int rotation = 4000;
		this._headMotor.rotate(rotation);
		this._headMotor.rotate(-(rotation*2));
		this._headMotor.rotate(rotation);
	}
	
	protected boolean _isCollisionLeft() {
		return this._leftBumper.isPressed() || this._leftMotor.isStopped();
	}
	
	protected boolean _isCollisionRight() {
		return this._rightBumper.isPressed() || this._rightMotor.isStopped();
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
	
	protected void _sleep(long ms) {
		try {
			Thread.sleep(ms);
		} catch (InterruptedException e) {}
	}
}
