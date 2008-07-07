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

import java.util.List;

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
public strictfp class Composite extends Particle {
	
	protected final SimpullCollection collection = new SimpullCollection();
	/** This is used as a reference point, helpful in determining the current rotation. */
	protected Particle firstParticleAdded;
	protected Particle centerParticle;

	private static final String UNABLE_TO_MODIFY_MESSAGE = "Cannot modify a Composite after it has been marked complete.";
	private static final String ALREADY_COMPLETE_MESSAGE = "Cannot mark a Composite as complete that is already marked complete.";
	
	private Vector2f delta = new Vector2f();
	private boolean compositeCompleted;
	private float initialReferenceRotation;
	
	/**
	 * The center position of this object will be determined and set when all components are 
	 * added and {@link Composite#setCompositeCompleted(boolean)} is called.  
	 * @param isFixed
	 * @param mass the total mass of the {@link Composite}, distributed among all {@link IPhysicsObject} added
	 */
	public Composite(boolean isFixed, float mass) {
		super(0f, 0f, isFixed, mass, 0f, 0f);
		setCollidable(false);
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
		compositeCompleted = true;
		final Particle compositeParticle = this;
		for (Particle particle : collection.getParticles()) {
			// Pass all collision events up from the elements to this top level particle.
			// TODO This should also include collidable constraints
			particle.eventAddHandler(Events.CollisionEvent.class.getName(), new EventHandler<Events.CollisionEvent>() {
				@Override
				public void handle(CollisionEvent event) {
					compositeParticle.eventAttemptNotify(event);
				}
			});
			particle.eventAddHandler(Events.FirstCollisionEvent.class.getName(), new EventHandler<Events.FirstCollisionEvent>() {
				@Override
				public void handle(Events.FirstCollisionEvent event) {
					compositeParticle.eventAttemptNotify(event);
				}
			});
		}
		setupCenterParticle();
		initialReferenceRotation = getRelativeAngle(getCenter(), firstParticleAdded.position);
		setMass(getMass());
		position.x = getX();
		position.y = getY();
		// TODO Does this need to take place for elasticity and friction too?
	}
	
	@Override
	public void addForce(IForce force) {
		centerParticle.addForce(force);
		// TODO Distribute?
	}

	@Override
	public void setElasticity(float elasticity) {
		super.setElasticity(elasticity);
		// TODO Distribute
	}

	@Override
	public void setFriction(float friction) {
		super.setFriction(friction);
		// TODO Distribute
	}

	@Override
	public void setMass(float mass) {
		super.setMass(mass);
		if (collection != null) {
			// TODO maybe make a generic method for distributing a value and pass a visitor to know what is distributed
			float massSum = 0f;
			float compositeMass = getMass(); // the mass passed into the constructor
			for (Particle particle : collection.getParticles()) {
				massSum += particle.getMass();
			}
			for (Particle particle : collection.getParticles()) {
				particle.setMass((particle.getMass() / massSum) * compositeMass);
			}
		}
	}

	@Override
	public void setMultisample(int m) {
		// TODO Auto-generated method stub
		super.setMultisample(m);
	}

	@Override
	public void setPosition(float x, float y) {
		centerParticle.setPosition(x, y);
	}

	@Override
	public void setSolid(boolean isSolid) {
		// TODO Auto-generated method stub
		super.setSolid(isSolid);
	}

	@Override
	public void setVelocity(Vector2f velocity) {
		centerParticle.setVelocity(velocity);
	}

	@Override
	public void setX(float x) {
		centerParticle.setX(x);
	}

	@Override
	public void setY(float y) {
		centerParticle.setY(y);
	}

	@Override
	public float getRotation() {
		// TODO make sure this is in the range 0 - 2pi
		return getRelativeAngle(getCenter(), firstParticleAdded.position) - initialReferenceRotation;
	}

	@Override
	public void setRotation(float rotation) {
		rotateBy(rotation - getRotation(), getCenter());
	}
	
	/**
	 * @return a copy of the position of the particle (i.e. changes to the object returned do not reflect).
	 * You can alter the position of a particle three ways: change its position, set
	 * its velocity, or apply a force to it. Setting the position of a non-fixed 
	 * particle is not the same as setting its fixed property to true. A particle held
	 * in place by its position will behave as if it's attached there by a 0 length
	 * spring constraint. 
	 */
	@Override
	public Vector2f getPosition() {
		return getCenter();
	}
	
	@Override
	public float getX() {
		return centerParticle.position.x;
	}
	
	@Override
	public float getY() {
		return centerParticle.position.y;
	}
	
	/** @return the fixed state of the Composite. */	
	public boolean getFixed() {
		for (Particle particle : collection.getParticles()) {
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
		for (Particle particle : collection.getParticles()) {
			particle.setFixed(isFixed);	
		}
	}
	
	@Override
	public void init() {
		super.init();
		collection.init();
	}

	@Override
	public void cleanup() {
		super.cleanup();
		collection.cleanup();
	}

	public void add(IConstraint... newConstraints) throws IllegalStateException {
		if (compositeCompleted) {
			throw new IllegalStateException(UNABLE_TO_MODIFY_MESSAGE);
		}
		collection.add(newConstraints);
	}

	public void add(Particle... newParticles) throws IllegalStateException {
		if (compositeCompleted) {
			throw new IllegalStateException(UNABLE_TO_MODIFY_MESSAGE);
		}
		collection.add(newParticles);
		if (firstParticleAdded == null) {
			firstParticleAdded = newParticles[0];
		}
	}

	public void remove(IConstraint constraint) throws IllegalStateException {
		if (compositeCompleted) {
			throw new IllegalStateException(UNABLE_TO_MODIFY_MESSAGE);
		}
		collection.remove(constraint);
	}

	public void remove(Particle particle) throws IllegalStateException {
		if (compositeCompleted) {
			throw new IllegalStateException(UNABLE_TO_MODIFY_MESSAGE);
		}
		collection.remove(particle);
	}
	
	public List<IConstraint> getConstraints() {
		return collection.getConstraints();
	}

	public List<Particle> getParticles() {
		return collection.getParticles();
	}
	
	public boolean isCompositeCompleted() {
		return compositeCompleted;
	}
	
	public Particle getCenterParticle() {
		return centerParticle;
	}
	
	boolean getHasParent() {
		return collection.getHasParent();
	}	

	void setHasParent(boolean hasParent) {
		collection.setHasParent(hasParent);
	}	
	
	void integrate(float dt2) {
		collection.integrate(dt2);
		centerParticle.setRotation(getRotation()); // This currently is only needed to support the simpull2core view sync
	}		
	
	void satisfyConstraints() {
		collection.satisfyConstraints();
	}			
	
	void checkInternalCollisions() {
		collection.checkInternalCollisions();
	}

	void checkCollisionsVsCollection(SimpullCollection group) {
		collection.checkCollisionsVsCollection(group);
	}
	
	SimpullCollection getCollection() {
		return collection;
	}
	
	/** @return the relative angle in radians from the center to the point */
	private float getRelativeAngle(Vector2f center, Vector2f point) {
		delta.x = point.x - center.x;
		delta.y = point.y - center.y;
		return (float)Math.atan2(delta.y, delta.x);
	}
	
	/** Rotates the Composite to an angle specified in radians, around a given center */
	private void rotateBy(float angleRadians, Vector2f center) {
		for (Particle particle : collection.getParticles()) {
			Vector2f diff = particle.getCenter();
			diff.x -= center.x;
			diff.y -= center.y;
			float radius = (float)Math.sqrt(diff.x * diff.x + diff.y * diff.y);
			float angle = getRelativeAngle(center, particle.getCenter()) + angleRadians;
			particle.setX((float)(Math.cos(angle) * radius) + center.x);
			particle.setY((float)(Math.sin(angle) * radius) + center.y);
		}
	}
	
	private void setupCenterParticle() {
		Vector2f centerPoint = calculateCenter();
		centerParticle = new Particle(centerPoint.x, centerPoint.y, false, 0.1f, 0f, 0f);
		for (Particle particle : collection.getParticles()) {
			SimpleSpring connectorToCenter = new SimpleSpring(centerParticle, particle, 1f, false, 1f, 1f, false);
			collection.add(connectorToCenter);
		}
	}
	
	private Vector2f calculateCenter() {
		Vector2f center = new Vector2f();
		
		float minX = Float.MAX_VALUE;
		float maxX = Float.MIN_VALUE;
		for (Particle particle : collection.getParticles()) {
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
		for (Particle particle : collection.getParticles()) {
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