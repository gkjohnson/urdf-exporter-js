import { Matrix3, Object3D, Vector3, Euler } from 'three';

class URDFBase extends Object3D {

	constructor( ...args ) {

		super( ...args );
		this.urdfNode = null;
		this.urdfName = '';

	}

	copy( source, recursive ) {

		super.copy( source, recursive );

		this.urdfNode = source.urdfNode;
		this.urdfName = source.urdfName;

		return this;

	}

}

class URDFCollider extends URDFBase {

	constructor( ...args ) {

		super( ...args );
		this.isURDFCollider = true;
		this.type = 'URDFCollider';

	}

}

class URDFVisual extends URDFBase {

	constructor( ...args ) {

		super( ...args );
		this.isURDFVisual = true;
		this.type = 'URDFVisual';

	}

}

class URDFInertialFrame {

	constructor() {

		this.position = new Vector3();
		this.rotation = new Euler();
		this.mass = 0;
		this.inertial = new Matrix3();

	}

}

class URDFLink extends URDFBase {

	constructor( ...args ) {

		super( ...args );
		this.isURDFLink = true;
		this.type = 'URDFLink';

		this.inertial = new URDFInertialFrame();

	}

}

class URDFLimit {

	constructor() {

		this.lower = 0;
		this.upper = 0;
		this.effort = 0;
		this.velocity = 0;

	}

}

class URDFJoint extends URDFBase {

	constructor( ...args ) {

		super( ...args );

		this.isURDFJoint = true;
		this.type = 'URDFJoint';

		this.jointType = 'fixed';
		this.axis = new Vector3( 1, 0, 0 );
		this.limit = new URDFLimit();

	}

}

class URDFMimicJoint extends URDFJoint {

	constructor( ...args ) {

		super( ...args );
		this.type = 'URDFMimicJoint';
		this.mimicJoint = null;
		this.offset = 0;
		this.multiplier = 1;

	}

}

class URDFRobot extends URDFLink {

	constructor( ...args ) {

		super( ...args );
		this.isURDFRobot = true;

		this.robotName = '';

	}

}

export { URDFRobot, URDFLink, URDFJoint, URDFMimicJoint, URDFVisual, URDFCollider };
