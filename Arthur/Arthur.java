import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.Sound;
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
	protected static final int LEFT = 3;
	protected static final int RIGHT = 4;
	protected static final int COLLISION_BOTH = 5;
	
	// This is the value, in tachos, of a 90 degress rotation from 0 tachos
	protected static final int DEGREES_90 = 90; // 4900;
	
	protected static final int MAXIMUM_DESIRED_WALL_DISTANCE = 15;
	
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
	protected int _wallDistance;
	
	public Arthur() {
		this._leftBumper = new TouchSensor(SensorPort.S4);
		this._rightBumper = new TouchSensor(SensorPort.S1);
		
		this._headSensor = new UltrasonicSensor(SensorPort.S3);
		
		this._leftMotor = Motor.A;
		this._rightMotor = Motor.C;
		this._headMotor = Motor.B;
		
		this._headMotor.resetTachoCount();
		this._headMotor.setSpeed(400);
		
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
			// Check distance from
			int distance_front = this._scanPoint(this._degreesToTacho(0));
			
			if (this._stalled) {
				this._log("Stall detected");
				this._travel(-1);
			} else if (distance_front < 15) {
				this._stop();
				this._log("That front wall", "is getting a", "little close");
				this._actCollisionBoth();
			} else if (this._isCollisionLeft() || this._isCollisionRight()) {
				this._stop();
				
				this._sleep(500);
				
				if (this._isCollisionBoth()) {
					this._log("COLLISION: BOTH");
					this._wallDistance = -1;
					this._wallSide = COLLISION_NONE;
					this._actCollisionBoth();
				} else if (this._isCollisionLeft()) {
					this._log("COLLISION: LEFT");
					this._wallDistance = -1;
					this._wallSide = LEFT;
					this._actCollisionSingle(LEFT);
				} else if (this._isCollisionRight()) {
					this._log("COLLISION: RIGHT");
					this._wallDistance = -1;
					this._wallSide = RIGHT;
					this._actCollisionSingle(RIGHT);
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
		
		int left_distance = this._scanPoint(this._degreesToTacho(90));
		
		this._wallSide = COLLISION_NONE;
		this._wallDistance = -1;
		
		int angle = this._tachoToDegrees(this._scanRange(DEGREES_90, true));
		this._log("ANGLE: " + angle);
		
		int right_distance = this._scanPoint(this._degreesToTacho(-90));
		
		this._log("Startup Act", "L: " + left_distance, "R: " + right_distance, "Angle: " + angle); this._sleep(1000);
		
		if (Math.abs(left_distance - this.MAXIMUM_DESIRED_WALL_DISTANCE) <= 10) {
			this._log("Following left"); this._sleep(1000);
			this._wallSide = LEFT;
			this._wallDistance = left_distance;
			this._actFollow();
		} else if (Math.abs(right_distance - this.MAXIMUM_DESIRED_WALL_DISTANCE) <= 10) {
			this._log("Following right"); this._sleep(1000);
			this._wallSide = RIGHT;
			this._wallDistance = right_distance;
			this._actFollow();
		} else {
			this._log("Moving through", "angle to wall"); this._sleep(1000);
			this._rotate(angle);
			this._forward();
		}
	}
	
	/**
	 * Action: Called when Arthur knows there's a wall on one side and should follow it
	 */
	protected int _heading = 0;
	protected void _actFollow() {
		this._stop();
		int side_modifier = (this._wallSide == LEFT) ? 1 : -1;
		
		int corner_distance = 10;
		int error_distance = 2;
		int lost_distance = 50;
		
		boolean pause = false;
		float travel_distance = 0.5f;
		int travel_rotate = 15;
		
		int distance = this._scanPoint(this._degreesToTacho(side_modifier * 90 + this._heading));
		int distance_change = distance - this._wallDistance;
		
		this._log("Dist: " + distance, "Dist': " + this._wallDistance, "Change: " + (distance - this._wallDistance) ,"Need: " + (distance - this.MAXIMUM_DESIRED_WALL_DISTANCE)); if (pause) { this._sleep(2500); }
		
		if (this._wallDistance < 0) {
			// We had no previous reading, so just carry on
			this._travel(travel_distance);
		} else if (distance_change > lost_distance) {
			// We think we've just plain lost the wall
			this._actStartup();
			return;
		} else if (distance_change > corner_distance) {
			// We think we've hit a corner
			this._rotate(side_modifier * travel_rotate * 2);
			this._travel(travel_distance * 2);
		} else if (distance_change > 0 && distance < this.MAXIMUM_DESIRED_WALL_DISTANCE) {
			// Moving away, but not at the mark yet
			
			this._log("Moving away", "Not at mark", "Hold"); if (pause) { this._sleep(2500); }
			
			this._travel(travel_distance);
		} else if (distance_change >= 0 && distance > this.MAXIMUM_DESIRED_WALL_DISTANCE) {
			// Moving away, but we've missed the mark

			this._log("Moving away", "Missed mark", "Correct"); if (pause) { this._sleep(2500); }
			
			// Use the distance change to make the turn sharper if there's a greater jump
			this._rotate(side_modifier * (travel_rotate + distance_change));
			this._travel(travel_distance);
		} else if (distance_change < 0 && distance < this.MAXIMUM_DESIRED_WALL_DISTANCE) {
			// Moving towards, we've missed the mark
			
			this._log("Moving towards", "Missed mark", "Correct"); if (pause) { this._sleep(2500); }
			
			// Use the distance change to make the turn sharper if there's a greater jump
			this._rotate(side_modifier * (-travel_rotate - distance_change));
			this._travel(travel_distance);
		} else if (distance_change < 0 && distance > this.MAXIMUM_DESIRED_WALL_DISTANCE) {
			// Moving towards, not at mark yet
			
			this._log("Moving towards", "Not at mark", "Hold"); if (pause) { this._sleep(2500); }
			
			this._travel(travel_distance);
		} else if (distance_change == 0 || distance == this.MAXIMUM_DESIRED_WALL_DISTANCE) {
			// We're there, hold
			
			this._log("No movement", "Mark", "Hold"); if (pause) { this._sleep(2500); }
			
			this._travel(travel_distance);
		}
		
		this._wallDistance = distance;
		
//		if (this._wallDistance < 0) {
//			// Note the distance to the wall when we start following it
//			this._wallDistance = distance;
//			this._heading = 0; // We know we're parallel
//		} else if (this._wallDistance > lost_distance && distance > lost_distance) {
//			// The wall is too far, we're lost, reset
//			this._actStartup();
//			return;
//		} else if (distance > this._wallDistance + corner_distance) {
//			// Lost the wall, probably a corner, move into it
//			this._rotate(side_modifier * 45);
//			this._wallDistance = -1;
//		} else if (distance - this.DESIRED_WALL_DISTANCE > 0) {
//		//} else if (distance > this._wallDistance + error_distance || distance > this._wallDistance - error_distance) {
//			// Moving away from the wall, correct it
//			this._log("Scan point: " + side_modifier * 90, "Distance: " + distance, "Relative: " + Math.abs(distance - this.DESIRED_WALL_DISTANCE), "Need move towards");
//			this._rotate(side_modifier * 3);
//			this._wallDistance = distance;
//			this._heading -= 10;
//		} else if (distance - this.DESIRED_WALL_DISTANCE < 0) {
//		//} else if (distance < this._wallDistance + error_distance || distance < this._wallDistance - error_distance) {
//			// Moving closer to the wall, correct it
//			this._log("Scan point: " + side_modifier * 90, "Distance: " + distance, "Relative: " + Math.abs(distance - this.DESIRED_WALL_DISTANCE), "Need move away");
//			this._rotate(side_modifier * -3);
//			this._wallDistance = distance;
//			this._heading += 10;
//		}
		
//		this._travel(travel_distance);
	}
	
	/**
	 * Action: Called when we have a collision on both bumpers
	 */
	protected void _actCollisionBoth() {
		int left_distance = this._scanPoint(this._degreesToTacho(90));
		int right_distance = this._scanPoint(this._degreesToTacho(-90));
		
		this._log("LEFT: " + left_distance, "RIGHT: " + right_distance);
		
		if (left_distance < 10 && right_distance < 10) {
			this._travel(-1);
			this._actCollisionBoth();
		} else {
			int angle = 90;
			this._wallSide = RIGHT;
			
			if (left_distance < right_distance) {
				angle *= -1;
				this._wallSide = LEFT;
			}
			
			this._travel(-0.5f);
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
		if (side == LEFT) {
			this._log("ACT: COLLISION L");
			side_modifier = -1;
		} else if (side == RIGHT) {
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
		
		if (rotation_angle > -7 && rotation_angle < 7) {
			// Nearest obstacle is parallel to the robot and shouldn't have
			// collided, therefore ignore scan data
			rotation_angle = side_modifier * 15;
			this._log("SCAN INCONCLUSIVE");
		} else {
			// Compensate for oversteer
			rotation_angle *= 0.9;
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
		int closestTacho = 0;
		int measurement = 255;
		
		if (this._shouldQuit()) { this._actFinish(); }
		
		int current_tacho = this._headMotor.getTachoCount();
		int initial_tacho = tacho - current_tacho;
		
		this._headMotor.rotate(initial_tacho, true);
		while (this._headMotor.isMoving()) {
			measurement = this._headSensor.getDistance();
			if (measurement < closestDistance) {
				closestDistance = measurement;
				closestTacho = this._headMotor.getTachoCount();
				
				this._log("SCAN: " + closestDistance + "@" + this._tachoToDegrees(closestTacho) + "DEG");
			}
		}
		
		if (this._shouldQuit()) { this._actFinish(); }
		
		if (both) {
			this._headMotor.rotate(-tacho);
			
			if (this._shouldQuit()) { this._actFinish(); }
			
			this._headMotor.rotate(-tacho, true);
			while (this._headMotor.isMoving()) {
				measurement = this._headSensor.getDistance();
				if (measurement < closestDistance) {
					closestDistance = measurement;
					closestTacho = this._headMotor.getTachoCount();
					
					this._log("SCAN: " + closestDistance + "@" + this._tachoToDegrees(closestTacho) + "DEG");
				}
			}
			
			if (this._shouldQuit()) { this._actFinish(); }

			this._headMotor.rotate(tacho, true);
		} else {
			this._headMotor.rotate(-tacho, true);
		}
		
		if (this._shouldQuit()) { this._actFinish(); }
		
		return closestTacho;
	}
	
	
	/**
	 * Measure the distance (in cm I think) until an object at a specified tacho
	 * 
	 * @param tacho
	 * @return
	 */
	protected int _scanPoint(int tacho) {
	
		// Adjust the rotation angle if the head is not straight
		// (saves waiting for the head to return if we need it on the same side)
		this._headMotor.stop();
		int current_tacho = this._headMotor.getTachoCount();
		if (current_tacho != 0) {
			tacho = tacho - current_tacho;
		}
		
		this._headMotor.rotate(tacho);
		int distance = this._headSensor.getDistance();;
		
		// Try three times to get a sensible reading
		for (int i = 0; i < 3; i++) {
			if (distance < 255) {
				return distance;
			} else {
				this._sleep(50);
				distance = this._headSensor.getDistance();
			}
		}
		
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
	
	/**
	 * Reset the head to look straight
	 */
	protected void _resetHead() {
		this._headMotor.stop();
		
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
