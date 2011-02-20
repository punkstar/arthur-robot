import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.TouchSensor;
import lejos.nxt.UltrasonicSensor;
import lejos.robotics.navigation.Pilot;
import lejos.robotics.navigation.TachoPilot;

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
	
	protected boolean _moving = false;
	protected boolean _stalled = false;
	
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
		
		this._startCheckStalledThread();
		
		while(!this._shouldQuit()) {
			if (this._stalled) {
				this._log("Stall detected");
				this._travel(-1);
			} else if (this._isCollisionLeft() || this._isCollisionRight()) {
				this._stop();
				
				this._sleep(500);
				
				if (this._isCollisionBoth()) {
					this._log("COLLISION: BOTH");
					this._actCollisionBoth();
				} else if (this._isCollisionLeft()) {
					this._log("COLLISION: LEFT");
					this._actCollisionSingle(COLLISION_LEFT);
				} else if (this._isCollisionRight()) {
					this._log("COLLISION: RIGHT");
					this._actCollisionSingle(COLLISION_RIGHT);
				} else {
					this._log("COLLISION: NONE!?");
					this._forward();
				}
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
		
		this._rotate(angle);
		this._forward();
	}
	
	/**
	 * Action: Called when we have a collision on both bumpers
	 */
	protected void _actCollisionBoth() {
		int left_distance = this._scanPoint(this._degreesToTacho(90));
		int right_distance = this._scanPoint(this._degreesToTacho(-90));
		
		this._log("LEFT: " + left_distance, "RIGHT: " + right_distance);
		
		if (left_distance < 30 && right_distance < 30) {
			this._travel(-1);
			this._actCollisionBoth();
		} else {
			int angle = 90;
			
			if (left_distance < right_distance) {
				angle *= -1;
			}
			
			this._rotate(angle);
			this._forward();
		}
	}
	
	/**
	 * Action: Called when we have a collision on a single bumper
	 * 
	 * @param side The side the collision occurred, can be Arthur.COLLISION_LEFT or Arthur.COLLISION_RIGHT
	 */
	protected void _actCollisionSingle(int side) {
		int side_modifier = 0;
		if (side == COLLISION_LEFT) {
			this._log("ACT: COLLISION L");
			side_modifier = -1;
		} else if (side == COLLISION_RIGHT) {
			this._log("ACT: COLLISION R");
			side_modifier = 1;
		} else {
			// Single collision called, but the collision side parameter was
			// not passed - this should never happen
			this._log("COLLISION ERROR:", "Act: Single collision", "Status: No collision");
			this._sleep(5000);
			this._actFinish();
		}
		
		int collision_tacho = this._scanRange(this._degreesToTacho(side_modifier * -90), false);
		int rotation_angle = this._tachoToDegrees(side_modifier * this._degreesToTacho(90) + collision_tacho);
		
		if (rotation_angle > -3 && rotation_angle < 3) {
			// Nearest obstacle is parallel to the robot and shouldn't have
			// collided, therefore ignore scan data
			rotation_angle = side_modifier * 15;
			this._log("SCAN INCONCLUSIVE");
		}
		
		this._travel(-0.5f);
		this._log("ROTATING " + rotation_angle + "deg");
		this._rotate(rotation_angle);
		
		this._forward();
	}
	
	/**
	 * Action: Called when we want to finish our run.
	 * 
	 * Resets the head to the initial heading, then quits.
	 */
	protected void _actFinish() {
		this._stop();
		
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

			// Don't both waiting for the head to get back to the correct position, don't block
			this._headMotor.rotate(tacho, true);
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
	
		int return_tacho = -1 * tacho;
		if (this._headMotor.isMoving()) {
			// Adjust the rotation angle if the head is not straight
			// (saves waiting for the head to return if we need it on the same side)
			this._headMotor.stop();
			int current_tacho = this._headMotor.getTachoCount();
			if (current_tacho != 0) {
				tacho = tacho - current_tacho;
			}
		}
		
		int distance;
		
		this._headMotor.rotate(tacho);
		distance = this._headSensor.getDistance();
		this._headMotor.rotate(return_tacho, true);
		
		return distance;
	}
	
	/**
	 * Pilot wrapper for travelling set distances, blocking while doing so.
	 * @param wheel_rotations
	 */
	protected void _travel(float wheel_rotations) {
		this._moving = true;
		this._pilot.travel(wheel_rotations);
		this._moving = false;
	}
	
	/**
	 * Pilot wrapper for rotating a specific angle.  Negative = clockwise.
	 * @param angle
	 */
	protected void _rotate(int angle) {
		this._moving = true;
		this._pilot.rotate(angle);
		this._moving = false;
	}
	
	/**
	 * Pilot wrapper for turning the motors on, forwards.
	 */
	protected void _forward() {
		this._moving = true;
		this._pilot.forward();
	}
	
	/**
	 * Pilot wrapper for turning the motors on, in reverse.
	 */
	protected void _backward() {
		this._moving = true;
		this._pilot.backward();
	}
	
	/**
	 * Pilot wrapper for turning the motors off.
	 */
	protected void _stop() {
		this._moving = false;
		this._pilot.stop();
	}
	
	/**
	 * Start a thread to check whether the motors are stalled at all, setting an instance variable if so.
	 * 
	 * @FIXME Doesn't appear to actually be working at the moment..
	 */
	protected void _startCheckStalledThread() {
		Thread t = new Thread() {
			public void run() {
				while (true) {
					_stalled = false;
					
					// Check sequentially three times in a row for a stall.
					if (_isStalled()) {
						_sleep(1000);
						if (_isStalled()) {
							_sleep(1000);
							if(_isStalled()) {
								_stalled = true;
								_sleep(1000);
							}
						}
					}
				}
			}
			
			protected boolean _isStalled() {
				return _moving && _leftMotor.isStopped() && _rightMotor.isStopped();
			}
		};
		
		t.start();
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
	
	protected int _degreesToTacho(int degrees) {
		return (degrees / 90) * DEGREES_90;
	}
	
	private void __blockWhileHeadMoving() {
		this._log("BLOCK: HEAD MOVING");
		while (this._headMotor.isMoving()) {}
	}
	
	/**
	 * A helper to log messages to the string.  As many parameters as you like, but you'll be capped to six lines on the screen.
	 * 
	 * @param messages
	 * @return Always returns true, so it can be used in conditionals
	 */
	protected boolean _log(String ... messages) {
		LCD.clear();
		LCD.drawString("ARTHUR SAYS:", 0, 0);
		
		int i = 1;
		for(String message : messages) {
			LCD.drawString(message, 0, i++);
		}
		
		return true;
	}
}
