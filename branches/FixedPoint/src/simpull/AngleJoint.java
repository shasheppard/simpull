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

/** An Angular Constraint between 3 Particles */
public class AngleJoint extends SimpleSpring {
	
	public Particle particle3;
	
	private SpringParticle collisionParticle23;
	private int minAngle;
	private int maxAngle;
	private int minBreakAngle;
	private int maxBreakAngle;
	private boolean isBroken;
	
	public AngleJoint(
			Particle particle1, 
			Particle particle2,
			Particle particle3,
			int minAngle,
			int maxAngle,
			int minBreakAngle,
			int maxBreakAngle,
			int stiffness,
			boolean isCollidable,
			int rectHeight,
			int rectScale,
			boolean scaleToLength/*= false*/) {
		super(particle1, particle2, stiffness, isCollidable, rectHeight, rectScale, scaleToLength);
		this.particle3 = particle3;
		
		if (minAngle == FP.from(10)) {
			setMinAngle(getAcRadian());
			setMaxAngle(getAcRadian());
		} else {
			setMinAngle(minAngle);
			setMaxAngle(maxAngle);
		}
		setMinBreakAngle(minBreakAngle);
		setMaxBreakAngle(maxBreakAngle);
	}
	
	/** The current difference between the angle of particle1, particle2, and particle3 and a straight line (pi) */	
	public int getAcRadian() {
		int ang12 = FP.atan2(particle2.position.y - particle1.position.y, particle2.position.x - particle1.position.x);
		int ang23 = FP.atan2(particle3.position.y - particle2.position.y, particle3.position.x - particle2.position.x);
		
		int angDiff = ang12 - ang23;
		return angDiff;
	}
	
	/** @return true if the passed particle is one of the three particles attached to this AngularConstraint. */		
	@Override
	public boolean isConnectedTo(IPhysicsObject other) {
		return other == particle1
				|| other == particle2
				|| other == particle3;
	}		
	
	/** @return true if any connected particle's isFixed property is true. */
	@Override
	public boolean getFixed() {
		return particle1.isFixed
				&& particle2.isFixed
				&& particle3.isFixed;
	}
	
	@Override
	public void setCollidable(boolean isCollidable, int rectHeight, 
			int rectScale, boolean scaleToLength/*=false*/) {
		super.setCollidable(isCollidable, rectHeight, rectScale, scaleToLength);
		collisionParticle23 = null;
		if (isCollidable) {
			collisionParticle23 = new SpringParticle(particle2, particle3, this, rectHeight, rectScale, scaleToLength);			
		}
		
		// FIXME: There is no call to collisionParticle.init() like there is when a constraint is added to a group and therefore some things will not work 
	}
	
	public int getMinAngle() {
		return minAngle;
	}
	
	public void setMinAngle(int angle) {
		minAngle = angle;
	}
	
	public int getMaxAngle() {
		return maxAngle;
	}
	
	public void setMaxAngle(int angle) {
		maxAngle = angle;
	}
	
	public int getMinBreakAngle() {
		return minBreakAngle;
	}
	
	public void setMinBreakAngle(int angle) {
		minBreakAngle = angle;
	}
	
	public int getMaxBreakAngle() {
		return maxBreakAngle;
	}
	
	public void setMaxBreakAngle(int angle) {
		maxBreakAngle = angle;
	}
	
	@Override
	public void resolve() {
		if (isBroken) {
			return;
		}
		
		int ang12 = FP.atan2(
				particle2.position.y - particle1.position.y, 
				particle2.position.x - particle1.position.x);
		int ang23 = FP.atan2(
				particle3.position.y - particle2.position.y, 
				particle3.position.x - particle2.position.x);
		
		int angDiff = ang12 - ang23;
 		// make sure angle is within range for later calcs
		while (angDiff > FP.PI) {
			angDiff -= FP.TWO_PI;
		}
		while (angDiff < -FP.PI) {
			angDiff += FP.TWO_PI;
		}
		
		int invMass1 = particle1.getInvMass();
		int invMass2 = particle2.getInvMass();
		int sumInvMass = invMass1 + invMass2;
		
		int mult1 = (int) (((long) invMass1 << FP.FRACTION_BITS) / sumInvMass);
		// above replaces int mult1 = invMass1 / sumInvMass;
		
		int mult2 = (int) (((long) invMass2 << FP.FRACTION_BITS) / sumInvMass);
		// above replaces int mult2 = invMass2 / sumInvMass;
		
		int angChange = 0;
		
		
		int lowMid = (int) (((long) (maxAngle - minAngle) << FP.FRACTION_BITS) / FP.TWO);
		// above replaces float lowMid = (maxAngleFX - minAngleFX) / 2;

		int highMid = (int) (((long) (maxAngle + minAngle) << FP.FRACTION_BITS) / FP.TWO);
		// above replaces float highMid = (maxAngleFX + minAngleFX) / 2;
 		
		int breakAng = (int) (((long) (maxBreakAngle - minBreakAngle) << FP.FRACTION_BITS) / FP.TWO);
		// above replaces float breakAng = (maxBreakAngleFX - minBreakAngleFX) / 2;
		
 		int newDiff = highMid - angDiff;
 		// make sure angle is within range for later calcs
		while (newDiff > FP.PI) {
			newDiff -= FP.TWO_PI;
		}
		while (newDiff < -FP.PI) {
			newDiff += FP.TWO_PI;
		}
		
		if (newDiff > lowMid) {
			if (newDiff > breakAng) {
				int diff = newDiff - breakAng;
				isBroken = true;
				// TODO add this back when events are wanted 
				// eventAttemptNotify(new Events.AngleJointBreakEvent(diff));
				return;
			}
			angChange = newDiff - lowMid;
		} else if (newDiff < -lowMid) {
			if (newDiff < - breakAng) {
				int diff2 = newDiff + breakAng;
				isBroken = true;
				// TODO add this back when events are wanted 
				// eventAttemptNotify(new Events.AngleJointBreakEvent(diff2));
				return;
			}
			angChange = newDiff + lowMid;
		}
		
		int finalAng = (int) (((long) angChange * this.stiffness) >> FP.FRACTION_BITS) + ang12;
		// above replaces float finalAng = angChange * this.stiffness + ang12;
		
		int displaceX = particle1.position.x + (int) (((long) (particle2.position.x - particle1.position.x) * mult1) >> FP.FRACTION_BITS);
		// above replaces float displaceX = particle1.position.x + (particle2.position.x - particle1.position.x) * mult1;
		
		int displaceY = particle1.position.y + (int) (((long) (particle2.position.y - particle1.position.y) * mult1) >> FP.FRACTION_BITS);
		// above replaces float displaceY = particle1.position.y + (particle2.position.y - particle1.position.y) * mult1;
		
		int finalAngPlusPI = finalAng + FP.PI;
		particle1.position.x = displaceX + (int) (((long) FP.cos(finalAngPlusPI) * ((int) (((long) restLength * mult1) >> FP.FRACTION_BITS))) >> FP.FRACTION_BITS);
		// above replaces particle1.position.x = displaceX + (float)Math.cos(finalAng + MathUtil.PI) * restLength * mult1;
		
		particle1.position.y = displaceY + (int) (((long) FP.sin(finalAngPlusPI) * ((int) (((long) restLength * mult1) >> FP.FRACTION_BITS))) >> FP.FRACTION_BITS);
		// above replaces particle1.position.y = displaceY + (float)Math.sin(finalAng + MathUtil.PI) * restLength * mult1;
		
		particle2.position.x = displaceX + (int) (((long) FP.cos(finalAng) * ((int) (((long) restLength * mult2) >> FP.FRACTION_BITS))) >> FP.FRACTION_BITS);
		// above replaces particle2.position.x = displaceX + (float)Math.cos(finalAng) * restLength * mult2;
		
		particle2.position.y = displaceY + (int) (((long) FP.sin(finalAng) * ((int) (((long) restLength * mult2) >> FP.FRACTION_BITS))) >> FP.FRACTION_BITS);
		// above replaces particle2.position.y = displaceY + (float)Math.sin(finalAng) * restLength * mult2;	
	}
	
}