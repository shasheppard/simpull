/*
    Copyright (c) 2008, Shaun Curtis Sheppard
    All rights reserved.
    
    Redistribution and use in source and binary forms, with or without 
    modification, are permitted provided that the following conditions are met:

        * Redistributions of source code must retain the above copyright 
          notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright 
          notice, this list of conditions and the following disclaimer in the 
          documentation and/or other materials provided with the distribution.
        * Neither the name of Shaun Curtis Sheppard nor the names of its 
          contributors may be used to endorse or promote products derived from 
          this software without specific prior written permission.
    
    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
    POSSIBILITY OF SUCH DAMAGE.
*/

package simpull;
	
/**
 * A particle that simulates the behavior of a wheel.
 * A Wheel can be fixed but rotate freely.
 */ 
public class Wheel extends Circle {

	private Rim rim;
	private Vector2f tan = new Vector2f();	
	private Vector2f normSlip = new Vector2f();
	private Vector2f orientation = new Vector2f();
	private int traction;

	/**
	 * @param x The initial x position.
	 * @param y The initial y position.
	 * @param radius The radius of this particle.
	 * @param isFixed Determines if the particle is fixed or not. Fixed particles
	 * are not affected by forces or collisions and are good to use as surfaces.
	 * Non-fixed particles move freely in response to collision and forces.
	 * @param mass The mass of the particle
	 * @param elasticity The elasticity of the particle. Higher values mean more elasticity.
	 * @param friction The surface friction of the particle. 
	 * @param traction The surface traction of the particle.
	 */
	public Wheel(
			int x, 
			int y, 
			int radius, 
			boolean isFixed/*= false*/, 
			int mass/*= 1*/, 
			int elasticity/*= 0.3*/,
			int friction/*= 0*/,
			int traction/*= 1*/) {
		super(x,y,radius,isFixed, mass, elasticity, friction);
		rim = new Rim(radius, FP.TWO); 	
		setTraction(traction);
	}	

	/**
	 * The speed of the WheelParticle. You can alter this value to make the 
	 * WheelParticle spin.
	 */
	public int getSpeed() {
		return rim.getSpeed();
	}
	
	public void setSpeed(int speed) {
		rim.setSpeed(speed);
	}

	/**
	 * The angular velocity of the WheelParticle. You can alter this value to make the 
	 * WheelParticle spin.
	 */
	public int getAngularVelocity() {
		return rim.getAngularVelocity();
	}
	
	public void setAngularVelocity(int angularVelocity) {
		rim.setAngularVelocity(angularVelocity);
	}
	
	/**
	 * The amount of traction during a collision. This property controls how much traction is 
	 * applied when the WheelParticle is in contact with another particle. If the value is set
	 * to 0, there will be no traction and the WheelParticle will behave as if the 
	 * surface was totally slippery, like ice. Values should be between 0 and 1. 
	 * 
	 * Note that the friction property behaves differently than traction. If the surface 
	 * friction is set high during a collision, the WheelParticle will move slowly as if
	 * the surface was covered in glue.
	 */		
	public int getTraction() {
		return FP.ONE - traction;
	}

	public void setTraction(int traction) {
		this.traction = FP.ONE - traction;
	}
	
	@Override
	public void init() {
		cleanup();
	}

	/** @return the rotation of the wheel in radians. */
	@Override
	public int getRotation() {
		orientation.x = rim.position.x;
		orientation.y = rim.position.y;

		return FP.atan2(orientation.y, orientation.x) + FP.PI;
	} 

	@Override
	public void update(int dt2) {
		super.update(dt2);
		rim.update(dt2);
	}

	@Override
	void respondToCollision(Collision collision, Vector2f mtd, Vector2f velocity, Vector2f normal,
			int depth, int order) {
		super.respondToCollision(collision, mtd, velocity, normal, depth, order);
		
		int sign = FP.sign((int) (((long) depth * order) >> FP.FRACTION_BITS)); // returns -1 or 1 integer, not fixed point
		sign = FP.from(sign);
		// above replaces int sign = MathUtil.sign(depth * order);
		
		Vector2f surfaceNormal = new Vector2f((int) (((long) normal.x * sign) >> FP.FRACTION_BITS), (int) (((long) normal.y * sign) >> FP.FRACTION_BITS));
		// above replaces Vector2f surfaceNormal = new Vector2f(normal.x * sign, normal.y * sign);
		
		resolve(surfaceNormal);
	}
	
	/** Simulates torque/wheel-ground interaction */
	private void resolve(Vector2f surfaceNormal) {
		// this is the tangent vector at the rim particle
		tan.x = -rim.position.y;
		tan.y = rim.position.x;

		// normalize so we can scale by the rotational speed
		tan = tan.normalize();

		// velocity of the wheel's surface 
		int rimSpeed = rim.getSpeed();
		
		Vector2f wheelSurfaceVelocity = 
			new Vector2f((int) (((long) tan.x * rimSpeed) >> FP.FRACTION_BITS), (int) (((long) tan.y * rimSpeed) >> FP.FRACTION_BITS));
// above replaces 		Vector2f wheelSurfaceVelocity = 
//			new Vector2f(tan.x * rimSpeed, tan.y * rimSpeed);
		
		// the velocity of the wheel's surface relative to the ground
		Vector2f velocity = getVelocity();
		Vector2f combinedVelocity = 
			new Vector2f(velocity.x + wheelSurfaceVelocity.x, velocity.y + wheelSurfaceVelocity.y);
	
		// the wheel's combined velocity projected onto the contact normal
		int cp = (int) (((long) combinedVelocity.x * surfaceNormal.y) >> FP.FRACTION_BITS) - (int) (((long) combinedVelocity.y * surfaceNormal.x) >> FP.FRACTION_BITS);
		// above replaces float cp = combinedVelocity.x * surfaceNormal.y - combinedVelocity.y * surfaceNormal.x;

		// set the wheel's spin speed to track the ground
		tan.x = (int) (((long) tan.x * cp) >> FP.FRACTION_BITS);
		// above replaces tan.x *= cp;
		
		tan.y = (int) (((long) tan.y * cp) >> FP.FRACTION_BITS);
		// above replaces tan.y *= cp;
		
		Vector2f rmt = new Vector2f(rim.position.x - tan.x, rim.position.y - tan.y);
		rim.prevPosition.x = rmt.x;
		rim.prevPosition.y = rmt.y;

		// some of the wheel's torque is removed and converted into linear displacement
		int slipSpeed = (int) (((long) (FP.ONE - traction) * rimSpeed) >> FP.FRACTION_BITS);
		// above replaces float slipSpeed = (1 - traction) * rim.getSpeed();
		
		normSlip.x = (int) (((long) slipSpeed * surfaceNormal.y) >> FP.FRACTION_BITS);
		// above replaces normSlip.x = slipSpeed * surfaceNormal.y;
		
		normSlip.y = (int) (((long) slipSpeed * surfaceNormal.x) >> FP.FRACTION_BITS);
		// above replaces normSlip.y = slipSpeed * surfaceNormal.x;
		
		position.x += normSlip.x;
		position.y += normSlip.y;
		
		rim.setSpeed((int) (((long) rimSpeed * traction) >> FP.FRACTION_BITS));
		// above replaces rim.setSpeed(rim.getSpeed() * traction);	
	}

	/**
	 * The Rim is really just a second component of the wheel model.
	 * The rim particle is simulated in a coordsystem relative to the wheel's 
	 * center, not in worldspace.
	 */
	class Rim {
		Vector2f position;
		Vector2f prevPosition;
		Vector2f tan = new Vector2f();

		private int wheelRadius;
		private int angularVelocity;
		private int speed;
		private int maxTorque;
		
		
		public Rim(int radius, int maxTorque) {
			position = new Vector2f(radius, 0);
			prevPosition = new Vector2f(0, 0);
			
			speed = 0; 
			angularVelocity = 0;
			
			this.maxTorque = maxTorque; 	
			wheelRadius = radius;		
		}
		
		int getSpeed() {
			return speed;
		}
		
		void setSpeed(int speed) {
			this.speed = speed;
		}
		
		int getAngularVelocity() {
			return angularVelocity;
		}
		
		void setAngularVelocity(int angularVelocity) {
			this.angularVelocity = angularVelocity;
		}
		
		void update(int dt2) {
			//clamp torques to valid range
			speed = Math.max(-maxTorque, Math.min(maxTorque, speed + angularVelocity));

			//apply torque
			//this is the tangent vector at the rim particle
			tan.x = -position.y;
			tan.y =  position.x;

			//normalize so we can scale by the rotational speed
			int len = FP.sqrt((int) (((long) tan.x * tan.x) >> FP.FRACTION_BITS) + (int) (((long) tan.y * tan.y) >> FP.FRACTION_BITS));
			// above replaces float len = (float)Math.sqrt(tan.x * tan.x + tan.y * tan.y);
			
			tan.x = (int) (((long) tan.x << FP.FRACTION_BITS) / len);
			// above replaces tan.x /= len;
			
			tan.y = (int) (((long) tan.y << FP.FRACTION_BITS) / len);
			// above replaces tan.y /= len;

			position.x += (int) (((long) speed * tan.x) >> FP.FRACTION_BITS);
			// above replaces position.x += speed * tan.x;
			
			position.y += (int) (((long) speed * tan.y) >> FP.FRACTION_BITS);
			// above replaces position.y += speed * tan.y;

			int prevX = prevPosition.x;
			int prevY = prevPosition.y;
			int x = prevPosition.x = position.x;
			int y = prevPosition.y = position.y;
			
			// Set position using the velocity multiplied by the damping
			position.x += (int) (((long) Simpull.damping * (x - prevX)) >> FP.FRACTION_BITS);
			// above replaces position.x += Simpull.damping * (x - prevX);
			
			position.y += (int) (((long) Simpull.damping * (y - prevY)) >> FP.FRACTION_BITS);
			//above replaces position.y += Simpull.damping * (y - prevY);	

			// hold the rim particle in place
			int clen = (int) (((long) position.x * position.x) >> FP.FRACTION_BITS) + (int) (((long) position.y * position.y) >> FP.FRACTION_BITS);
			// above replaces float clen = (float)Math.sqrt(position.x * position.x + position.y * position.y);
			
			int diff = (int) (((long) (clen - wheelRadius) << FP.FRACTION_BITS) / clen);
			// above replaces float diff = (clen - wheelRadius) / clen;

			position.x -= (int) (((long) position.x * diff) >> FP.FRACTION_BITS);
			// above replaces position.x -= position.x * diff;
			
			position.y -= (int) (((long) position.y * diff) >> FP.FRACTION_BITS);
			// above replaces position.y -= position.y * diff;
		}
		
	}

}


