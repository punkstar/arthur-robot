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
	protected static final int SIDE_LEFT = 3;
	protected static final int SIDE_RIGHT = 4;
	protected static final int COLLISION_BOTH = 5;
	
	// This is the value, in tachos, of a 90 degress rotation from 0 tachos
	protected static final int DEGREES_90 = 120;
	
	protected static final int MOTOR_TURN_SPEED = 360;
	protected static final int MOTOR_SCAN_SPEED = 90;
	
	protected static final int DESIRED_WALL_DISTANCE = 15;
	
	protected Pilot _pilot;
	
	protected TouchSensor _leftBumper;
	protected TouchSensor _rightBumper;
	
	protected UltrasonicSensor _headSensor;
	
	protected Motor _leftMotor;
	protected Motor _rightMotor;
	protected Motor _headMotor;
	
	protected boolean _moving = false;
	protected boolean _stalled = false;
	
	protected int _wallSide;
	
	public Arthur() {
		this._leftBumper = new TouchSensor(SensorPort.S4);
		this._rightBumper = new TouchSensor(SensorPort.S1);
		
		this._headSensor = new UltrasonicSensor(SensorPort.S3);
		
		this._leftMotor = Motor.A;
		this._rightMotor = Motor.C;
		this._headMotor = Motor.B;
		
		this._headMotor.resetTachoCount();
		this._headMotor.setSpeed(MOTOR_TURN_SPEED);
		
		this._pilot = new TachoPilot(0.56f, 1.18f, this._leftMotor, this._rightMotor);
	}
	
	/**
	 * The main deliberation method.
	 * 
	 * Here's where we decide what we're going to be doing based on bumper sensor information.
	 */
	public void deliberate() {
		this._actStartup();
		
		// @FIXME This isn't wokring properly
		//this._startCheckStalledThread();
		
		while(!this._shouldQuit()) {
			this._actCheckFront();
			
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
					this._actCollisionSingle(SIDE_LEFT);
				} else if (this._isCollisionRight()) {
					this._log("COLLISION: RIGHT");
					this._actCollisionSingle(SIDE_RIGHT);
				} else {
					this._log("COLLISION: NONE!?");
					this._forward();
				}
			} else if (this._wallSide != COLLISION_NONE) {
				// We know there's a wall on one side - follow it
				this._actFollow();
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
		
		this._stop();
		
		this._wallSide = COLLISION_NONE;
		
		int angle = this._tachoToDegrees(this._scanRange(DEGREES_90, true));
		this._log("ANGLE: " + angle);
		
		int distance = this._scanPoint(this._tachoToDegrees(angle));
		
		if (distance < DESIRED_WALL_DISTANCE + 3) {
			// Next to a wall - align to it and follow it
			int side_modifier = (angle > 0) ? -1 : 1;
			this._wallSide = (side_modifier < 0) ? SIDE_LEFT : SIDE_RIGHT;
			this._rotate((90 + side_modifier * angle) * side_modifier);
		} else {
			// No walls near - move to the nearest obstacle
			this._rotate(angle);
		}
		
		this._forward();
	}
	
	/**
	 * Action: Called when moving forward to avoid bumping into a wall.
	 */
	protected void _actCheckFront() {
		int distance = this._scanPoint(0);
		
		if (distance < DESIRED_WALL_DISTANCE) {
			// An obstacle ahead - act as if it was hit
			this._log("Wall in front!");
			this._stop();
			this._actCollisionBoth();
		}
	}
	
	/**
	 * Action: Called when Arthur knows there's a wall on one side and should follow it
	 */
	protected void _actFollow() {
		int side_modifier = (this._wallSide == SIDE_LEFT) ? 1 : -1;
		
		int corner_distance = 10;
		int error_distance = 2;
		
		int distance = this._scanPoint(this._degreesToTacho(side_modifier * 90));
		
		if (distance != DESIRED_WALL_DISTANCE) {
			this._stop();
			// Repeat the check to make sure we got a good scan
			int repeat = this._scanPoint(this._degreesToTacho(side_modifier * 90));
			
			if (distance - DESIRED_WALL_DISTANCE > corner_distance &&
				repeat - DESIRED_WALL_DISTANCE > corner_distance) {
				// The wall suddenly ended, probably a corner
				this._turn(side_modifier * 2.0f, 90);
			} else if (repeat != distance) {
				// Either original or repeat scan failed - ignore them
				// We don't care about this for the corners as the wall is not there anyway
				this._log("Scan mismatch!");
			} else if (distance < DESIRED_WALL_DISTANCE - error_distance) {
				// Too close to a wall - move away
				this._rotate(side_modifier * -10);
			} else if (distance > DESIRED_WALL_DISTANCE + error_distance) {
				// Too far from a wall - move into it
				this._rotate(side_modifier * 10);
			} else {
				this._log("Following wall");
			}
		}
		
		this._forward();
	}
	
	/**
	 * Action: Called when we have a collision on both bumpers
	 */
	protected void _actCollisionBoth() {
		int left_distance = this._scanPoint(this._degreesToTacho(90));
		int right_distance = this._scanPoint(this._degreesToTacho(-90));
		
		this._log("LEFT: " + left_distance, "RIGHT: " + right_distance);
		
		if (this._wallSide == SIDE_LEFT && right_distance > 30) {
			// Continue following the wall on the left
			this._travel(-0.5f);
			this._rotate(-90);
			this._forward();
		} else if (this._wallSide == SIDE_RIGHT && left_distance > 30) {
			// Continue following the wall on the right
			this._travel(-0.5f);
			this._rotate(90);
			this._forward();
		} else if (left_distance < 10 && right_distance < 10) {
			// Seems like a dead end - move back
			this._wallSide = COLLISION_NONE;
			this._travel(-1);
			this._actCollisionBoth();
		} else {
			// Turn to the side where there's more space
			int angle = 90;
			this._wallSide = SIDE_RIGHT;
			
			if (left_distance < right_distance) {
				angle *= -1;
				this._wallSide = SIDE_LEFT;
			}
			
			this._travel(-0.5f);
			this._rotate(angle);
			this._forward();
		}
	}
	
	/**
	 * Action: Called when we have a collision on a single bumper
	 * 
	 * @param side The side the collision occurred, can be Arthur.SIDE_LEFT or Arthur.SIDE_RIGHT
	 */
	protected void _actCollisionSingle(int side) {
		int side_modifier = 0;
		if (side == SIDE_LEFT) {
			this._log("ACT: COLLISION L");
			side_modifier = -1;
		} else if (side == SIDE_RIGHT) {
			this._log("ACT: COLLISION R");
			side_modifier = 1;
		} else {
			// Single collision called, but the collision side parameter was
			// not passed - this should never happen
			this._log("COLLISION ERROR:", "Act: Single collision", "Status: No collision");
			this._sleep(5000);
			this._actFinish();
		}
		
		// Note that we have a wall on one side
		this._wallSide = side;
		
		int collision_tacho = this._scanRange(this._degreesToTacho(side_modifier * -90), false);
		int rotation_angle = this._tachoToDegrees(side_modifier * this._degreesToTacho(90) + collision_tacho);
		
		if (rotation_angle > -7 && rotation_angle < 7) {
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
		
		this._resetHead();
		
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
		int closestTachoStart = 0;
		int closestTachoEnd = 0;
		boolean recordTacho = false;
		int measurement = 255;
		
		this._resetHead();
		this._headMotor.setSpeed(MOTOR_SCAN_SPEED);
		
		this._headMotor.rotate(tacho, true);
		while (this._headMotor.isMoving()) {
			measurement = this._headSensor.getDistance();
			if (measurement < closestDistance) {
				closestDistance = measurement;
				closestTachoStart = this._headMotor.getTachoCount();
				closestTachoEnd = closestTachoStart;
				recordTacho = true;
				
			} else if (measurement > closestDistance) {
				recordTacho = false;
			}
			if (recordTacho) {
				closestTachoEnd = this._headMotor.getTachoCount();
				this._log("SCAN: " + closestDistance + "@" + this._tachoToDegrees(closestTachoEnd) + "DEG");
			}
		}
		
		if (both) {
			this._headMotor.rotate(-tacho);
			
			recordTacho = false;
			
			this._headMotor.rotate(-tacho, true);
			while (this._headMotor.isMoving()) {
				measurement = this._headSensor.getDistance();
				if (measurement < closestDistance) {
					closestDistance = measurement;
					closestTachoStart = this._headMotor.getTachoCount();
					closestTachoEnd = closestTachoStart;
					
				} else if (measurement > closestDistance) {
					recordTacho = false;
				}
				if (recordTacho) {
					closestTachoEnd = this._headMotor.getTachoCount();
					this._log("SCAN: " + closestDistance + "@" + this._tachoToDegrees(closestTachoEnd) + "DEG");
				}
			}

			this._headMotor.rotate(tacho, true);
		} else {
			this._headMotor.rotate(-tacho, true);
		}
		
		return closestTachoStart + (closestTachoEnd - closestTachoStart) / 2;
	}
	
	
	/**
	 * Measure the distance (in cm I think) until an object at a specified tacho
	 * 
	 * @param tacho
	 * @return
	 */
	protected int _scanPoint(int tacho) {
		this._headMotor.setSpeed(MOTOR_TURN_SPEED);
	
		// Adjust the rotation angle if the head is not straight
		// (saves waiting for the head to return if we need it on the same side)
		this._headMotor.stop();
		int current_tacho = this._headMotor.getTachoCount();
		if (current_tacho != 0) {
			tacho = tacho - current_tacho;
		}
		
		int distance;
		
		this._headMotor.rotate(tacho);
		distance = this._headSensor.getDistance();
		
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
	 * Pilot wrapper for travelling in an arc.
	 *
	 * @param radius The radius of the arc. Negative = turn right.
	 * @param distance The distance (in degrees) to travel along the arc. Negative = backward.
	 */
	protected void _turn(float radius, float distance) {
		this._pilot.arc(radius, distance);
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
	
	/**
	 * Reset the head to look straight
	 */
	protected void _resetHead() {
		this._headMotor.stop();
		this._headMotor.setSpeed(MOTOR_TURN_SPEED);
		
		int angle = this._headMotor.getTachoCount();
		angle = -angle;
		
		if (angle != 0) {
			this._log("Head reset: "+ this._tachoToDegrees(angle) + "DEG");
			this._headMotor.rotate(angle);
		}
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
