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

import simpull.events.EventHandler;
import simpull.events.sample.Events;
import simpull.events.sample.Events.CollisionEvent;
	
/**
 * The Composite class can contain Particles, and Constraints.
 * The most common use is to create a complex physics object.
 * The Composite can be rotated all as one around a single point. 
 * Composites can be added to a parent Group, along with Particles and Constraints.  
 * Members of a Composite are not checked for collision with one another, internally.
 */ 
public strictfp class Composite extends SimpullCollection implements IPhysicsObject {
	
	/** This is used as a reference point, helpful in determining the current rotation. */
	protected Particle firstParticleAdded;
	/** This is used in conjunction with firstParticleAdded to apply rotational force (angular velocity) */
	protected Particle middleParticleAdded;
	protected final Particle centerParticle;

	private static final String UNABLE_TO_MODIFY_MESSAGE = "Cannot modify a Composite after it has been marked complete.";
	private static final String ALREADY_COMPLETE_MESSAGE = "Cannot mark a Composite as complete that is already marked complete.";
	private static final String CANNOT_ADD_INCOMPLETE_MESSAGE = "Cannot add a Composite that is not marked complete.";
	
	private Vector2f delta = new Vector2f();
	private boolean compositeCompleted;
	private float initialReferenceRotation;
	private float angularVelocity;
	/** When no coordinates are given, the center point will be calculated based on the layout of the particles added */
	private boolean calculateCenterPoint;
	private float mass;
	
	/**
	 * The center position of this object will be determined and set when all components are 
	 * added and {@link Composite#setCompositeCompleted(boolean)} is called.  
	 * @param isFixed
	 * @param mass the total mass of the {@link Composite}, distributed among all {@link IPhysicsObject} added
	 */
	public Composite(boolean isFixed, float mass) {
		centerParticle = new Particle(0, 0, false, mass, 0f, 0f);
		this.mass = mass;
		calculateCenterPoint = true;
		centerParticle.setCollidable(false);
	}
	
	public Composite(float centerX, float centerY, boolean isFixed, float mass) {
		centerParticle = new Particle(centerX, centerY, false, mass, 0f, 0f);
		this.mass = mass;
		calculateCenterPoint = false;
		centerParticle.setCollidable(false);
	}
	
	/**
	 * After adding all {@link Particle} and {@link IConstraint} instances
	 * to this composite, it must be marked as complete to allow further processing.
	 * @param finishers a way for child classes to perform additional processing when the composite is complete.
	 * 			These are called before any internal processing takes place.
	 * @throws IllegalStateException if already marked as complete
	 */
	public final void setCompositeCompleted(Finisher... finishers) throws IllegalStateException {
		if (compositeCompleted) {
			throw new IllegalStateException(ALREADY_COMPLETE_MESSAGE);
		}
		for (Finisher finisher : finishers) {
			finisher.finish(this);
		}
		for (Particle particle : particles) {
			// Pass all collision events up from the elements to this top level particle.
			// TODO This should also include collidable constraints
			particle.eventAddHandler(Events.CollisionEvent.class.getName(), new EventHandler<Events.CollisionEvent>() {
				@Override
				public void handle(CollisionEvent event) {
					centerParticle.eventAttemptNotify(event);
				}
			});
			particle.eventAddHandler(Events.FirstCollisionEvent.class.getName(), new EventHandler<Events.FirstCollisionEvent>() {
				@Override
				public void handle(Events.FirstCollisionEvent event) {
					centerParticle.eventAttemptNotify(event);
				}
			});
		}
		setupCenter();
		compositeCompleted = true;
		initialReferenceRotation = getRelativeAngle(centerParticle.position, firstParticleAdded.position);
		setMass(mass);
		// TODO Does this need to take place for elasticity and friction too?
	}
	
	public void setAngularVelocity(float angularVelocity) {
		this.angularVelocity = angularVelocity;
	}
	
	public void setElasticity(float elasticity) {
		// TODO Distribute
	}

	public void setFriction(float friction) {
		// TODO Distribute
	}

	/**
	 * Distributes the mass passed throughout each particle in this collection.
	 * The center particle is assigned the exact value passed in.
	 * @param mass
	 */
	public void setMass(float mass) {
		// TODO maybe make a generic method for distributing a value and pass a visitor to know what is distributed
		float massSum = 0f;
		for (Particle particle : particles) {
			massSum += particle.getMass();
		}
		float centerMass = mass / (float)particles.size();
		float nonCenterMass = mass - centerMass;
		for (Particle particle : particles) {
			particle.setMass((particle.getMass() / massSum) * nonCenterMass);
		}
		centerParticle.setMass(centerMass);
	}
	
	public float getMass() {
		return mass;
	}

	public float getRotation() {
		// TODO make sure this is in the range 0 - 2pi
		return getRelativeAngle(centerParticle.position, firstParticleAdded.position) - initialReferenceRotation;
	}

	public void setRotation(float rotation) {
		rotateBy(rotation - getRotation(), centerParticle.position);
	}
	
	/** @return the fixed state of the Composite. */	
	public boolean getFixed() {
		for (Particle particle : particles) {
			if (!particle.getFixed()) {
				return false;	
			}
		}
		return true;
	}

	/**
	 * @param isFixed the fixed state of the Composite. Setting this value to true or false will
	 * set all of this Composite's component particles to that value. Getting this 
	 * value will return false if any of the component particles are not fixed.
	 */	
	public void setFixed(boolean isFixed) {
		for (Particle particle : particles) {
			particle.setFixed(isFixed);	
		}
	}
	
	@Override
	public void init() {
		super.init();
		centerParticle.init();
	}

	@Override
	public void cleanup() {
		super.cleanup();
		centerParticle.cleanup();
	}

	@Override
	public void add(IConstraint... newConstraints) throws IllegalStateException {
		if (compositeCompleted) {
			throw new IllegalStateException(UNABLE_TO_MODIFY_MESSAGE);
		}
		super.add(newConstraints);
	}

	@Override
	public void add(Particle... newParticles) throws IllegalStateException {
		if (compositeCompleted) {
			throw new IllegalStateException(UNABLE_TO_MODIFY_MESSAGE);
		}
		super.add(newParticles);
		if (firstParticleAdded == null) {
			firstParticleAdded = newParticles[0];
		}
	}
	
	/**
	 * Add one/many {@link Composite} instances to this one.
	 * @param newComposites
	 * @throws IllegalStateException if this is already marked as complete, or if adding one that is not marked complete.
	 */
	public void add(Composite... newComposites) throws IllegalStateException {
		if (compositeCompleted) {
			throw new IllegalStateException(UNABLE_TO_MODIFY_MESSAGE);
		}
		for (Composite newComposite : newComposites) {
			if (!newComposite.compositeCompleted) {
				throw new IllegalStateException(CANNOT_ADD_INCOMPLETE_MESSAGE);
			}
			for (Particle newParticle : newComposite.particles) {
				add(newParticle);
			}
			for (IConstraint newConstraint : newComposite.constraints) {
				add(newConstraint);
			}
		}
	}

	public void remove(IConstraint constraint) throws IllegalStateException {
		if (compositeCompleted) {
			throw new IllegalStateException(UNABLE_TO_MODIFY_MESSAGE);
		}
		remove(constraint);
	}

	public void remove(Particle particle) throws IllegalStateException {
		if (compositeCompleted) {
			throw new IllegalStateException(UNABLE_TO_MODIFY_MESSAGE);
		}
		remove(particle);
	}
	
	public void remove(Composite composite) throws IllegalStateException {
		if (compositeCompleted) {
			throw new IllegalStateException(UNABLE_TO_MODIFY_MESSAGE);
		}
		// FIXME add the impl
	}
	
	public boolean isCompositeCompleted() {
		return compositeCompleted;
	}
	
	public Particle getCenterParticle() {
		return centerParticle;
	}

	@Override
	void integrate(float dt2) {
		super.integrate(dt2);
		centerParticle.update(dt2);
		// handle rotational velocity
		if (angularVelocity != 0f) {
			rotateBy(angularVelocity * dt2, centerParticle.position);
		}
		
		// The following is deemed unnecessary after making Composite extends SimpullCollection again, b/c things changed.
		//centerParticle.setRotation(getRotation()); // This currently is only needed to support the simpull2core view sync
		//centerParticle.update(dt2);
	}
	
	/* * This method integrates the particle, not taking into account velocity - just global/local forces. * /
	private void updateCenterParticle(float dt2) {
		Vector2f forces = centerParticle.getForces();
		// accumulate global forces
		float invMass = centerParticle.getInvMass();
		for (IForce iForce : Simpull.forces) {
			Vector2f vForce = iForce.getValue(invMass);
			forces.x += vForce.x;
			forces.y += vForce.y;
		}
		Vector2f positionBefore = new Vector2f(centerParticle.position.x, centerParticle.position.y);
		float dt2damping = dt2 * Simpull.damping;
		forces.x *= dt2damping;
		forces.y *= dt2damping;
		centerParticle.position.x += forces.x;
		centerParticle.position.y += forces.y;
		// clear all the forces out, the appropriate forces are added each step
		forces.x = 0;
		forces.y = 0;
	}
	*/
	
	/** @return the relative angle in radians from the center to the point */
	private float getRelativeAngle(Vector2f center, Vector2f point) {
		delta.x = point.x - center.x;
		delta.y = point.y - center.y;
		return (float)Math.atan2(delta.y, delta.x);
	}
	
	/** Rotates the Composite to an angle specified in radians, around a given center */
	private void rotateBy(float angleRadians, Vector2f center) {
		for (Particle particle : particles) {
			Vector2f diff = particle.getCenter();
			diff.x -= center.x;
			diff.y -= center.y;
			float radius = (float)Math.sqrt(diff.x * diff.x + diff.y * diff.y);
			float angle = getRelativeAngle(center, particle.getCenter()) + angleRadians;
			// To make sure the composite's velocity is not affected by changes to posotion from rotating,
			// we take the velocity before updating the position and set it back afterward.
			Vector2f velocity = particle.getVelocity();
			particle.position.x = (float)(Math.cos(angle) * radius) + center.x;
			particle.position.y = (float)(Math.sin(angle) * radius) + center.y;
			particle.setVelocity(velocity);
		}
	}
	
	private void setupCenter() {
		if (calculateCenterPoint) {
			Vector2f centerPoint = calculateCenter();
			centerParticle.position.x = centerPoint.x;
			centerParticle.position.y = centerPoint.y;
		}
		// Just have to set this in order to keep the velocity at 0, b/c this sets the prevPosition...thus, keeps the velocity at 0
		centerParticle.setPosition(centerParticle.position.x, centerParticle.position.y);
		for (Particle particle : particles) {
			SimpleSpring connectorToCenter = new SimpleSpring(centerParticle, particle, 1f, false, 1f, 1f, false);
			add(connectorToCenter);
		}
	}
	
	private Vector2f calculateCenter() {
		Vector2f center = new Vector2f();
		
		float minX = Float.MAX_VALUE;
		float maxX = Float.MIN_VALUE;
		for (Particle particle : particles) {
			if (particle.position.x > maxX) {
				maxX = particle.position.x;
			}
			if (particle.position.x < minX) {
				minX = particle.position.x;
			}
		}
		center.x = ((maxX - minX) / 2f) + minX;

		float minY = Float.MAX_VALUE;
		float maxY = Float.MIN_VALUE;
		for (Particle particle : particles) {
			if (particle.position.y > maxY) {
				maxY = particle.position.y;
			}
			if (particle.position.y < minY) {
				minY = particle.position.y;
			}
		}
		center.y = ((maxY - minY) / 2f) + minY;
		
		return center;
	}
	
	public interface Finisher {
		public void finish(Composite composite);
	}
	
}