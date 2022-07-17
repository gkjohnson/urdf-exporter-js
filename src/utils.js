import { Euler, Quaternion, Vector3, Matrix4 } from 'three';

const _euler = new Euler();
const _quaternion = new Quaternion();
const _scale = new Vector3();
const _position = new Vector3();
const _matrix = new Matrix4();

export function repeatChar( char, count ) {

	let result = '';
	for ( let i = 0; i < count; i ++ ) {

		result += char;

	}

	return result;

}

// returns the transform of root relative to parent
export function getRelativeOriginNode( root, parent ) {

	_matrix.copy( parent.matrixWorld ).invert().premultiply( root.matrixWorld );
	return getOriginNode( _matrix );

}

// returns the origin node for the given matrix
export function getOriginNode( matrix ) {

	matrix.decompose( _position, _quaternion, _scale );
	_euler.setFromQuaternion( _quaternion, 'ZYX' );

	return `<origin xyz="${ _position.x } ${ _position.y } ${ _position.z }" rpy="${ _euler.x } ${ _euler.y } ${ _euler.z }"/>\n`;

}

// recursively traverses the root until reaching a URDF node. Stops if the callback returns "true".
export function traverseImmediateMeaningfulNodes( root, cb ) {

	const children = root.children;
	for ( let i = 0, l = children.length; i < l; i ++ ) {

		const c = children[ i ];
		if (
			c.isURDFLink ||
            c.isURDFJoint ||
            c.isURDFCollider ||
            c.isURDFVisual
		) {

			if ( cb( c ) === true ) {

				break;

			}

		}

	}

}

// recursively traverse the parent until a callback returns "true" or the robot root is found.
export function traverseParents( root, cb ) {

	let curr = root.parent;
	while ( curr ) {

		if ( cb( curr ) === true ) {

			break;

		}

		if ( curr.isURDFRobot ) {

			break;

		}

		curr = curr.parent;

	}

}

// returns the first found child link.
export function getChildLink( root ) {

	let result = null;
	traverseImmediateMeaningfulNodes( root, node => {

		if ( node.isURDFLink ) {

			result = node;
			return true;

		}

	} );

	return result;

}

// returns the first found parent link.
export function getParentLink( root ) {

	let result = null;
	traverseParents( root, node => {

		if ( node.isURDFLink ) {

			result = node;
			return true;

		}

	} );

	return result;

}

// returns the first found parent joint.
export function getParentJoint( root ) {

	let result = null;
	traverseParents( root, node => {

		if ( node.isURDFJoint ) {

			result = node;
			return true;

		}

	} );

	return result;

}

// returns a matrix i, j element of the matrix
function getMatrixElement( matrix, i, j ) {

	const { elements } = matrix;
	return elements[ i + j * 4 ];

}

// returns the inertia matrix
export function getInertiaNode( matrix ) {

	const xx = getMatrixElement( matrix, 0, 0 );
	const yy = getMatrixElement( matrix, 1, 1 );
	const zz = getMatrixElement( matrix, 2, 2 );

	const xy = getMatrixElement( matrix, 0, 1 );
	const xz = getMatrixElement( matrix, 0, 2 );
	const yz = getMatrixElement( matrix, 1, 2 );

	return `<inertia ixx=${ xx } iyy=${ yy } izz=${ zz } ixy=${ xy } ixz=${ xz } iyz=${ yz } />\n`;

}

