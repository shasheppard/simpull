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

import java.util.ArrayList;
import java.util.List;
	
/**
 * The Group class can contain {@link Particle}, {@link IConstraint}, and {@link Composite}. Groups can be 
 * assigned to be checked for collision with other Groups and/or internally. 
 */ 
public class Group extends SimpullCollection {
	
	List<Composite> composites;
	List<Group> collideWithGroups;
	
	private static final String CANNOT_ADD_INCOMPLETE_COMPOSITE_MESSAGE = "Cannot add an incomplete Composite to a Group.";
	
	private boolean collideInternal;
	
	/**
	 * The Group class is the main organizational class for APE. Once groups are 
	 * created and populated with particles, constraints, and composites, they are 
	 * added to the Simpull. Groups may contain particles, constraints, and 
	 * composites. Composites may only contain particles and constraints.
	 */
	public Group(boolean collideInternal/*= false*/) {
		composites = new ArrayList<Composite>();
		collideWithGroups = new ArrayList<Group>();
		setCollideInternal(collideInternal);
	}
	
	/** Initializes every member by calling each member's init method. */
	@Override
	public void init() {
		super.init();
		for (Composite composite : composites) {
			composite.init();	
		}
	}
	
	/** 
	 * @param composites The {@link Composite}(s) to be added.
	 * @throws IllegalStateException when one of the newComposites is not completed
	 */
	public void add(Composite... newComposites) throws IllegalStateException {
		for (Composite composite : newComposites) {
			if (composite.isCompositeCompleted()) {
				composites.add(composite);
				composite.setHasParent(true);
				if (getHasParent()) {
					composite.init();
				}
			} else {
				throw new IllegalStateException(CANNOT_ADD_INCOMPLETE_COMPOSITE_MESSAGE);
			}
		}
	}

	/** @param composite the Composite to be removed. */
	public void remove(Composite composite) {
		composites.remove(composite);
		composite.setHasParent(false);
		composite.cleanup();
	}
	
	/** @param collidableGroups {@link Group} instances to be checked for collision against this one. */
	public void registerCollidable(Group... collidableGroups) {
		for (Group collidableGroup : collidableGroups) {
			collideWithGroups.add(collidableGroup);
		}
	}

	/** @param collidableGroup a {@link Group} to remove from the collidable list of this Group. */
	public void unregisterCollidable(Group collidableGroup) {
		collideWithGroups.remove(collidableGroup);
	}

	public boolean getCollideInternal() {
		return collideInternal;
	}
	
	/**
	 * Determines if the members of this Group are checked for collision with one 
	 * another.
	 */
	public void setCollideInternal(boolean collideInternal) {
		this.collideInternal = collideInternal;
	}
	
	/**
	 * Cleans up everything in this Group.
	 * This method is called automatically when an Group is removed
	 * from Simpull.
	 */
	@Override
	public void cleanup() {
		super.cleanup();
		for (Composite composite : composites) {
			composite.cleanup();	
		}
	}
	
	@Override
	void integrate(float dt2) {
		super.integrate(dt2);
		for (Composite composite : composites) {
			composite.integrate(dt2);
		}						
	}
	
	@Override
	void satisfyConstraints() {
		super.satisfyConstraints();
		for (Composite composite : composites) {
			composite.satisfyConstraints();
		}				
	}
	
	void checkCollisions() {
		if (getCollideInternal()) {
			checkCollisionGroupInternal();
		}
		for (Group group : collideWithGroups) {
			if (group == null) {
				continue;
			}
			checkCollisionVsGroup(group);
		}
	}
	
	private void checkCollisionGroupInternal() {
		// check collisions not in composites
		checkInternalCollisions();
		
		int clen = composites.size();
		for (int j = 0; j < clen; ++j) {
			Composite jComposite = composites.get(j);
			// check against non composite particles and constraints in this group
			jComposite.checkCollisionsVsCollection(this);
			
			// check against every other composite in this Group
			for (int i = j + 1; i < clen; ++i) {
				Composite iComposite = composites.get(i);
				jComposite.checkCollisionsVsCollection(iComposite);
			}
		}
	}
	
	private void checkCollisionVsGroup(Group group) {
		// check particles and constraints not in composites of either group
		checkCollisionsVsCollection(group);
		
		Composite groupComposite;
		int clen = composites.size();
		int gclen = group.composites.size();
		
		for (int i = 0; i < clen; ++i) {
			// check against the particles and constraints of g
			Composite myComposite = composites.get(i);
			if (myComposite == null) {
				continue;
			}
			myComposite.checkCollisionsVsCollection(group);
			
			// check against composites of g
			for (int j = 0; j < gclen; ++j) {
				groupComposite = group.composites.get(j);
				if (groupComposite == null) {
					continue;
				}
				myComposite.checkCollisionsVsCollection(groupComposite);
			}
		}
		
		// check particles and constraints of this group vs the composites of g
		for (int j = 0; j < gclen; ++j) {
			groupComposite = group.composites.get(j);
			if (groupComposite == null) {
				continue;	
			}
			checkCollisionsVsCollection(groupComposite);
		}
	}
	
}