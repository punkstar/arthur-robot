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
	
	// This is the value, in tachos, of a 90 degress rotation from 0 tachos
	protected static final int DEGREES_90 = 4900;
	
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
	
	/**
	 * The main deliberation method.
	 * 
	 * Here's where we decide what we're going to be doing based on bumper sensor information.
	 */
	public void deliberate() {
		this._actStartup();
		
		while(!this._shouldQuit()) {
			if (this._isCollisionLeft() || this._isCollisionRight()) {
				this._pilot.stop();
				int collision = COLLISION_NONE;
				
				this._sleep(500);
				
				if (this._isCollisionBoth()) {
					this._log("COLLISION: BOTH");
					collision = COLLISION_BOTH;
					this._actCollisionBoth();
				} else if (this._isCollisionLeft()) {
					this._log("COLLISION: LEFT");
					collision = COLLISION_LEFT;
					this._actCollisionSingle(collision);
				} else if (this._isCollisionRight()) {
					this._log("COLLISION: RIGHT");
					collision = COLLISION_RIGHT;
					this._actCollisionSingle(collision);
				} else {
					this._log("COLLISION: NONE!?");
					this._pilot.forward();
				}
			} else {
				this._log("MOVING");
			}
		}
		
		this._actFinish();
	}
	
	/**
	 * Action: Called when starting up Arthur
	 * 
	 * We aim to find the nearest object, based on a 180 scan of our surroundings, then move towards it.
	 */
	protected void _actStartup() {
		this._log("ACT: STARTUP");
		
		int angle = this._tachoToDegrees(this._scanRange(DEGREES_90, true));
		this._log("ANGLE: " + angle);
		
		this._pilot.rotate(angle);
		this._pilot.forward();
	}
	
	/**
	 * Action: Called when we have a collision on both bumpers
	 */
	protected void _actCollisionBoth() {
	}
	
	/**
	 * Action: Called when we have a collision on a single bumper
	 * 
	 * @param side The side the collision occurred, can be Arthur.COLLISION_LEFT or Arthur.COLLISION_RIGHT
	 */
	protected void _actCollisionSingle(int side) {
		int angle = 0;
		if (side == COLLISION_LEFT) {
			angle = -30;
			this._log("ACT: COLLISION L");
		} else if (side == COLLISION_RIGHT) {
			angle = 30;
			this._log("ACT: COLLISION R");
		} else {
			// FAIL
			this._log("ACT: OMGWTFBBQ!");
		}
		
		this._pilot.travel(-1); // in wheel rotations
		this._pilot.rotate(angle);
		
		this._pilot.forward();
	}
	
	/**
	 * Action: Called when we want to finish our run.
	 * 
	 * Resets the head to the initial heading, then quits.
	 */
	protected void _actFinish() {
		this._pilot.stop();
		
		this.__blockWhileHeadMoving();
		
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
	
	/**
	 * Perform a sweeping scan of a range ([-tacho, tacho] if both is set to true, else [0, tacho]), returning the
	 * tacho with the closest object.
	 * 
	 * @param tacho
	 * @param both If true scan from [-tacho,tacho], else [0,tacho]
	 * @return
	 */
	protected int _scanRange(int tacho, boolean both) {
		int closestDistance = 255;
		int closestTacho = 0;
		int measurement = 255;
		
		this.__blockWhileHeadMoving();
		
		this._headMotor.rotate(tacho, true);
		while (this._headMotor.isMoving()) {
			measurement = this._headSensor.getDistance();
			if (measurement < closestDistance) {
				closestDistance = measurement;
				closestTacho = this._headMotor.getTachoCount();
				
				this._log("SCAN: " + closestDistance + "@" + this._tachoToDegrees(closestTacho) + "DEG");
			}
		}

		this._headMotor.rotate(-tacho);
		
		if (both) {
			this._headMotor.rotate(-tacho, true);
			while (this._headMotor.isMoving()) {
				measurement = this._headSensor.getDistance();
				if (measurement < closestDistance) {
					closestDistance = measurement;
					closestTacho = this._headMotor.getTachoCount();
					
					this._log("SCAN: " + closestDistance + "@" + this._tachoToDegrees(closestTacho) + "DEG");
				}
			}

			this._headMotor.rotate(tacho);
		}
		
		return closestTacho;
	}
	
	/**
	 * Measure the distance (in cm I think) until an object at a specified tacho
	 * 
	 * @param tacho
	 * @return
	 */
	protected int _scanPoint(int tacho) {
		this.__blockWhileHeadMoving();
		
		int distance;
		
		this._headMotor.rotate(tacho);
		distance = this._headSensor.getDistance();
		this._headMotor.rotate(-tacho);
		
		return distance;
	}
	
	protected boolean _isCollisionLeft() {
		return this._leftBumper.isPressed();
	}
	
	protected boolean _isCollisionRight() {
		return this._rightBumper.isPressed();
	}
	
	protected boolean _isCollisionBoth() {
		return this._isCollisionLeft() && this._isCollisionRight();
	}
	
	/**
	 * A helper to log messages to the string
	 * 
	 * @param message
	 * @return Always returns true, so it can be used in conditionals
	 */
	protected boolean _log(String message) {
		LCD.clear();
		LCD.drawString("ARTHUR SAYS:", 0, 0);
		LCD.drawString(message, 0, 1);
		
		return true;
	}
	
	/**
	 * Decides whether or not we should quit, based on whether the escape button is pressed or not.
	 * @return
	 */
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
		return 90 * tacho / DEGREES_90;
	}
	
	private void __blockWhileHeadMoving() {
		this._log("BLOCK: HEAD MOVING");
		while (this._headMotor.isMoving()) {}
	}
}
