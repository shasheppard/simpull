package simpull;

public interface IPhysicsObject {

	/**
	 * Called when 
	 * 	a) the {@link Group} I belong to is added to {@link Simpull}
	 *  b) the {@link Composite} I am a part of is added to a {@link Group}
	 *  c) I am added to a {@link Composite} or {@link Group}
	 */
	public void init();
	
			
	/**
	 * Called when the {@link Group} I belong to is removed from {@link Simpull}
	 */
	public void cleanup();

}
