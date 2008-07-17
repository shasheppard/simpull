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
 * An Angular Constraint between 3 Particles
 */
public strictfp class AngleJoint extends SimpleSpring {
	
	public Particle particle3;
	
	private SpringParticle collisionParticle23;
	private float minAngle;
	private float maxAngle;
	private float minBreakAngle;
	private float maxBreakAngle;
	private boolean isBroken;
	
	public AngleJoint(
			Particle particle1, 
			Particle particle2,
			Particle particle3,
			float minAngle,
			float maxAngle,
			float minBreakAngle/*= -10*/,
			float maxBreakAngle/*= 10*/,
			float stiffness/*= .5*/,
			boolean isCollidable/*= false*/,
			float rectHeight/*= 1*/,
			float rectScale/*= 1*/,
			boolean scaleToLength/*= false*/) {
		super(particle1, particle2, stiffness, isCollidable, rectHeight, rectScale, scaleToLength);
		this.particle3 = particle3;
		
		if (minAngle == 10) {
			setMinAngle(getAcRadian());
			setMaxAngle(getAcRadian());
		} else {
			setMinAngle(minAngle);
			setMaxAngle(maxAngle);
		}
		setMinBreakAngle(minBreakAngle);
		setMaxBreakAngle(maxBreakAngle);
	}
	
	/**
	 * The current difference between the angle of particle1, particle2, and particle3 and a straight line (pi)
	 * 
	 */	
	public float getAcRadian() {
		float ang12 = (float)Math.atan2(particle2.position.y - particle1.position.y, particle2.position.x - particle1.position.x);
		float ang23 = (float)Math.atan2(particle3.position.y - particle2.position.y, particle3.position.x - particle2.position.x);
		
		float angDiff = ang12 - ang23;
		return angDiff;
	}
	
	/**
	 * Returns true if the passed particle is one of the three particles attached to this AngularConstraint.
	 */		
	@Override
	public boolean isConnectedTo(Particle particle) {
		return particle == particle1
				|| particle == particle2
				|| particle == particle3;
	}		
	
	/** @return true if any connected particle's isFixed property is true. */
	@Override
	public boolean getFixed() {
		return particle1.isFixed
				&& particle2.isFixed
				&& particle3.isFixed;
	}
	
	@Override
	public void setCollidable(boolean isCollidable, float rectHeight, 
			float rectScale, boolean scaleToLength/*=false*/) {
		super.setCollidable(isCollidable, rectHeight, rectScale, scaleToLength);
		collisionParticle23 = null;
		if (isCollidable) {
			collisionParticle23 = new SpringParticle(particle2, particle3, this, rectHeight, rectScale, scaleToLength);			
		}
		
		// FIXME: There is no call to collisionParticle.init() like there is when a constraint is added to a group and therefore some things will not work 
	}
	
	public float getMinAngle() {
		return minAngle;
	}
	
	public void setMinAngle(float angle) {
		minAngle = angle;
	}
	
	public float getMaxAngle() {
		return maxAngle;
	}
	
	public void setMaxAngle(float angle) {
		maxAngle = angle;
	}
	
	public float getMinBreakAngle() {
		return minBreakAngle;
	}
	
	public void setMinBreakAngle(float angle) {
		minBreakAngle = angle;
	}
	
	public float getMaxBreakAngle() {
		return maxBreakAngle;
	}
	
	public void setMaxBreakAngle(float angle) {
		maxBreakAngle = angle;
	}
	
	@Override
	public void resolve() {
		if (isBroken) {
			return;
		}
		float ang12 = (float)Math.atan2(
				particle2.position.y - particle1.position.y, 
				particle2.position.x - particle1.position.x);
		float ang23 = (float)Math.atan2(
				particle3.position.y - particle2.position.y, 
				particle3.position.x - particle2.position.x);
		
		float angDiff = ang12 - ang23;
 		// make sure angle is within range for later calcs
		while (angDiff > MathUtil.PI) {
			angDiff -= MathUtil.TWO_PI;
		}
		while (angDiff < -MathUtil.PI) {
			angDiff += MathUtil.TWO_PI;
		}
		
		float sumInvMass = particle1.getInvMass() + particle2.getInvMass();
		float mult1 = particle1.getInvMass() / sumInvMass;
		float mult2 = particle2.getInvMass() / sumInvMass;
		float angChange = 0;
		
		float lowMid = (maxAngle - minAngle) / 2;
		float highMid = (maxAngle + minAngle) / 2;
 		float breakAng = (maxBreakAngle - minBreakAngle) / 2;
		
 		float newDiff = highMid - angDiff;
 		// make sure angle is within range for later calcs
		while (newDiff > MathUtil.PI) {
			newDiff -= MathUtil.TWO_PI;
		}
		while (newDiff < -MathUtil.PI) {
			newDiff += MathUtil.TWO_PI;
		}
		
		if (newDiff > lowMid) {
			if (newDiff > breakAng) {
				float diff = newDiff - breakAng;
				isBroken = true;
				// TODO add this back when events are wanted 
				// eventAttemptNotify(new Events.AngleJointBreakEvent(diff));
				return;
			}
			angChange = newDiff - lowMid;
		} else if (newDiff < -lowMid) {
			if (newDiff < - breakAng) {
				float diff2 = newDiff + breakAng;
				isBroken = true;
				// TODO add this back when events are wanted 
				// eventAttemptNotify(new Events.AngleJointBreakEvent(diff2));
				return;
			}
			angChange = newDiff + lowMid;
		}
		
		float finalAng = angChange * this.stiffness + ang12;
		float displaceX = particle1.position.x + (particle2.position.x - particle1.position.x) * mult1;
		float displaceY = particle1.position.y + (particle2.position.y - particle1.position.y) * mult1;
		
		particle1.position.x = displaceX + (float)Math.cos(finalAng + MathUtil.PI) * restLength * mult1;
		particle1.position.y = displaceY + (float)Math.sin(finalAng + MathUtil.PI) * restLength * mult1;
		particle2.position.x = displaceX + (float)Math.cos(finalAng) * restLength * mult2;
		particle2.position.y = displaceY + (float)Math.sin(finalAng) * restLength * mult2;	
	}
	
}