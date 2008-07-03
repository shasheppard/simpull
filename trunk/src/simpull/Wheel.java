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
public strictfp class Wheel extends Circle {

	private Rim rim;
	private Vector2f tan;	
	private Vector2f normSlip;
	private Vector2f orientation;
	private float traction;

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
			float x, 
			float y, 
			float radius, 
			boolean isFixed/*= false*/, 
			float mass/*= 1*/, 
			float elasticity/*= 0.3*/,
			float friction/*= 0*/,
			float traction/*= 1*/) {
		super(x,y,radius,isFixed, mass, elasticity, friction);
		tan = new Vector2f(0,0);
		normSlip = new Vector2f(0,0);
		rim = new Rim(radius, 2); 	
		
		setTraction(traction);
		orientation = new Vector2f();
	}	

	/**
	 * The speed of the WheelParticle. You can alter this value to make the 
	 * WheelParticle spin.
	 */
	public float getSpeed() {
		return rim.getSpeed();
	}
	
	public void setSpeed(float speed) {
		rim.setSpeed(speed);
	}

	/**
	 * The angular velocity of the WheelParticle. You can alter this value to make the 
	 * WheelParticle spin.
	 */
	public float getAngularVelocity() {
		return rim.getAngularVelocity();
	}
	
	public void setAngularVelocity(float angularVelocity) {
		rim.setAngularVelocity(angularVelocity);
	}
	
	/**
	 * The amount of traction during a collision. This property controls how much traction is 
	 * applied when the WheelParticle is in contact with another particle. If the value is set
	 * to 0, there will be no traction and the WheelParticle will behave as if the 
	 * surface was totally slippery, like ice. Values should be between 0 and 1. 
	 * 
	 * <p>
	 * Note that the friction property behaves differently than traction. If the surface 
	 * friction is set high during a collision, the WheelParticle will move slowly as if
	 * the surface was covered in glue.
	 * </p>
	 */		
	public float getTraction() {
		return 1 - traction;
	}

	public void setTraction(float traction) {
		this.traction = 1 - traction;
	}
	
	/**
	 * Sets up the visual representation of this particle. This method is automatically called when 
	 * an particle is added to the engine.
	 */
	@Override
	public void init() {
		cleanup();
	}

	/**
	 * The rotation of the wheel in radians.
	 */
	@Override
	public float getRotation() {
		orientation.x = rim.position.x;
		orientation.y = rim.position.y;
		return (float)(Math.atan2(orientation.y, orientation.x) + Math.PI);
	} 

	/**
	 *
	 */		
	@Override
	public void update(float dt) {
		super.update(dt);
		rim.update(dt);
	}

	@Override
	void respondToCollision(Collision collision, Vector2f mtd, Vector2f velocity, Vector2f n,
			float d, int o) {
		super.respondToCollision(collision, mtd, velocity, n, d, o);
		int sign = MathUtil.sign(d * o);
		Vector2f surfaceNormal = new Vector2f(n.x * sign, n.y * sign);
		resolve(surfaceNormal);
	}
	
	/**
	 * Simulates torque/wheel-ground interaction
	 */
	private void resolve(Vector2f surfaceNormal) {
		// this is the tangent vector at the rim particle
		tan.x = -rim.position.y;
		tan.y = rim.position.x;

		// normalize so we can scale by the rotational speed
		tan = tan.normalize();

		// velocity of the wheel's surface 
		float rimSpeed = rim.getSpeed();
		Vector2f wheelSurfaceVelocity = 
			new Vector2f(tan.x * rimSpeed, tan.y * rimSpeed);
		
		// the velocity of the wheel's surface relative to the ground
		Vector2f velocity = getVelocity();
		Vector2f combinedVelocity = 
			new Vector2f(velocity.x + wheelSurfaceVelocity.x, velocity.y + wheelSurfaceVelocity.y);
	
		// the wheel's comb velocity projected onto the contact normal
		float cp = combinedVelocity.x * surfaceNormal.y - combinedVelocity.y * surfaceNormal.x;

		// set the wheel's spin speed to track the ground
		tan.x *= cp;
		tan.y *= cp;
		Vector2f rmt = new Vector2f(rim.position.x - tan.x, rim.position.y - tan.y);
		rim.prevPosition.x = rmt.x;
		rim.prevPosition.y = rmt.y;

		// some of the wheel's torque is removed and converted into linear displacement
		float slipSpeed = (1 - traction) * rim.getSpeed();
		normSlip.x = slipSpeed * surfaceNormal.y;
		normSlip.y = slipSpeed * surfaceNormal.x;
		position.x += normSlip.x;
		position.y += normSlip.y;
		rim.setSpeed(rim.getSpeed() * traction);	
	}

	/**
	 * The Rim is really just a second component of the wheel model.
	 * The rim particle is simulated in a coordsystem relative to the wheel's 
	 * center, not in worldspace.
	 */
	strictfp class Rim {
		Vector2f position;
		Vector2f prevPosition;

		private float wheelRadius;
		private float angularVelocity;
		private float speed;
		private float maxTorque;
		
		
		public Rim(float radius, float maxTorque) {
			position = new Vector2f(radius, 0);
			prevPosition = new Vector2f(0, 0);
			
			speed = 0; 
			angularVelocity = 0;
			
			this.maxTorque = maxTorque; 	
			wheelRadius = radius;		
		}
		
		float getSpeed() {
			return speed;
		}
		
		void setSpeed(float speed) {
			this.speed = speed;
		}
		
		float getAngularVelocity() {
			return angularVelocity;
		}
		
		void setAngularVelocity(float angularVelocity) {
			this.angularVelocity = angularVelocity;
		}
		
		void update(float dt) {
			//clamp torques to valid range
			speed = Math.max(-maxTorque, Math.min(maxTorque, speed + angularVelocity));

			//apply torque
			//this is the tangent vector at the rim particle
			float dx = -position.y;
			float dy =  position.x;

			//normalize so we can scale by the rotational speed
			float len = (float)Math.sqrt(dx * dx + dy * dy);
			dx /= len;
			dy /= len;

			position.x += speed * dx;
			position.y += speed * dy;		

			float ox = prevPosition.x;
			float oy = prevPosition.y;
			float px = prevPosition.x = position.x;		
			float py = prevPosition.y = position.y;		
			
			position.x += Simpull.damping * (px - ox);
			position.y += Simpull.damping * (py - oy);	

			// hold the rim particle in place
			float clen = (float)Math.sqrt(position.x * position.x + position.y * position.y);
			float diff = (clen - wheelRadius) / clen;

			position.x -= position.x * diff;
			position.y -= position.y * diff;
		}
		
	}

}


