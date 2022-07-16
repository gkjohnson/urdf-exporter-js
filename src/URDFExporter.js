import { Euler } from 'three';
import {
	getChildLink,
	getOriginNode,
	getParentJoint,
	getParentLink,
	getRelativeOriginNode,
	repeatChar,
	traverseImmediateMeaningfulNodes,
	traverseParents,
	getInertiaNode,
} from './utils.js';

// http://wiki.ros.org/urdf/XML/
const _euler = new Euler();
export class URDFExporter {

	constructor() {

		this.indent = '\t';
		this.processGeometryCallback = () => {

			return null;

		};

	}

	parse( root ) {

		const { indent, processGeometryCallback } = this;

		const indent1 = repeatChar( indent, 1 );
		const indent2 = repeatChar( indent, 2 );
		const indent3 = repeatChar( indent, 3 );
		const indent4 = repeatChar( indent, 4 );

		let result = '';
		if ( ! root.isURDFRobot ) {

			console.warn( 'URDFExporter: Root link is expected to be a URDFRobot instance.' );

		}

		// initialize the names of joints and links that are unnamed
		const nameMap = new Map();
		let linkIndex = 0;
		let jointIndex = 0;
		root.traverse( c => {

			if ( c.isURDFLink ) {

				if ( c.name ) {

					nameMap.set( c, c.name );

				} else {

					nameMap.set( c, `link_${ linkIndex ++ }` );

				}

			} else if ( c.isURDFJoint ) {

				if ( c.name ) {

					nameMap.set( c, c.name );

				} else {

					nameMap.set( c, `joint_${ jointIndex ++ }` );

				}

			}

		} );


		const linkNodes = [];
		const jointNodes = [];

		processLink( root );

		result += `<robot name="${ root.robotName || 'robot' }>`;

		result += linkNodes.join( '' );

		result += jointNodes.join( '' );

		result += '</robot>';

		return result;

		function processVisualContents( node ) {

			const relativeParent = getParentJoint( node ) || node;
			const children = node.children;
			let result = '';

			// if we have one child then it might be a geometric result
			if ( children.length === 1.0 && children[ 0 ].isMesh ) {

				const mesh = children[ 0 ];
				if ( mesh.geometry.isSphereGeometry ) {

					// sphere
					result += `${ indent4 }${ getRelativeOriginNode( mesh, relativeParent ) }`;

					const radius = mesh.geometry.parameters.radius * mesh.scale.x;
					result += `${ indent4 }<sphere radius="${ radius }" />`;

				} else if ( mesh.geometry.isBoxGeometry ) {

					// box
					result += `${ indent4 }${ getRelativeOriginNode( mesh, relativeParent ) }`;

					let { width, height, depth } = mesh.geometry.parameters;
					width *= mesh.scale.x;
					height *= mesh.scale.y;
					depth *= mesh.scale.z;

					result += `${ indent }<box size="${ width } ${ height } ${ depth }" />`;

				} else if ( mesh.geometry.isCylinderGeometry ) {

					// cylinder
					// TODO: include three.js rotation offset here
					result += `${ indent4 }${ getRelativeOriginNode( mesh, relativeParent ) }`;

					let { radiusTop, height } = mesh.geometry.parameters;
					radiusTop *= radiusTop * mesh.scale.x;
					height *= mesh.scale.y;

					result += `${ indent }<cylinder length="${ height }" radius="${ radiusTop }" />`;

				}

			}

			if ( result === '' ) {

				// mesh
				result += `${ indent4 }${ getRelativeOriginNode( node, relativeParent ) }`;

				const path = processGeometryCallback( node );
				if ( path !== null ) {

					result += `${ indent4 }<geometry><mesh filename="${ path }"/></geometry>`;

				}

			}

			return result;

		}


		function processLink( link ) {

			let result = '';
			result += `${ indent1 }<link name="${ nameMap.get( link ) }">`;

			// inertia
			const inertial = link.inertial;
			if ( inertial ) {

				const { origin, rotation, mass, inertia } = inertial;
				_euler.copy( rotation ).reorder( 'zyx' );

				result += `${ indent2 }<inertial>`;
				result += `${ indent3 }<origin xyz="${ origin.x } ${ origin.y } ${ origin.z }" rpy="${ _euler.x } ${ _euler.y } ${ _euler.z }" />`;
				result += `${ indent3 }<mass value="${ mass }/>`;
				result += `${ indent3 }${ getInertiaNode( inertia ) }`;
				result += `${ indent2 }</inertial>`;

			}

			// process any necessary child information
			traverseImmediateMeaningfulNodes( link, child => {

				if ( child.isURDFJoint ) {

					processJoint( child );

				} else if ( child.isURDFVisual ) {

					result += `${ indent3 }<visual`;
					if ( child.name ) {

						result += ` name="${ child.name }"`;

					}

					result += '/>';
					result += processVisualContents( child );
					result += `${ indent3 }<visual>`;

				} else if ( child.isURDFCollider ) {

					result += `${ indent3 }<collider`;
					if ( child.name ) {

						result += ` name="${ child.name }"`;

					}

					result += '/>';
					result += processVisualContents( child );
					result += `${ indent3 }<collider>`;

				} else {

					console.warn( `URDFExporter: Link "${ link.name }" cannot contain another link.` );

				}

			} );

			result += `${ indent1 }</link>`;
			linkNodes.push( result );

		}

		function processJoint( joint ) {

			// warn user of invalid structure
			let totalLinks = 0;
			let totalJoints = 0;
			let totalVisual = 0;
			let totalCollider = 0;
			traverseImmediateMeaningfulNodes( joint, child => {

				if ( child.isURDFJoint ) {

					totalJoints ++;

				} else if ( child.isURDFVisual ) {

					totalVisual ++;

				} else if ( child.isURDFCollider ) {

					totalCollider ++;

				} else if ( child.isURDFLink ) {

					totalLinks ++;

				}

			} );

			if ( totalLinks > 1 ) console.warn( `URDFExporter: too many links are children of Joint ${ joint.name }.` );
			if ( totalJoints > 0 ) console.warn( `URDFExporter: joints cannot be children of Joint ${ joint.name }.` );
			if ( totalVisual > 0 ) console.warn( `URDFExporter: visual nodes cannot be children of Joint ${ joint.name }.` );
			if ( totalCollider > 0 ) console.warn( `URDFExporter: collider nodes cannot be children of Joint ${ joint.name }.` );

			// find the relevant parent and child joints
			const parentLink = getParentLink( joint );
			const parentJoint = getParentJoint( joint );
			const childLink = getChildLink( joint );

			// construct teh node
			let result = '';
			result += `${ indent1 }<joint name="${ nameMap.get( joint ) }" type="${ joint.jointType || 'fixed' }">`;

			result += `${ indent2 }${ getRelativeOriginNode( joint, parentJoint || parentLink ) }`;

			result += `${ indent2 }<parent link="${ parentLink.name }"/>`;

			result += `${ indent2 }<child link="${ childLink.name }"/>`;

			if ( joint.jointType === 'revolute' || joint.jointType === 'continuous' || joint.jointType === 'prismatic' ) {

				const axis = joint.axis;
				result += `${ indent2 }<axis xyz="${ axis.x } ${ axis.y } ${ axis.z }"/>`;

			}

			// TODO: include more limits?
			if ( joint.limits ) {

				const limits = joint.limits;
				result += `${ indent2 }<limit effort="${ limits.effort || 0 }" velocity="${ limits.velocity }"`;
				if ( joint.jointType !== 'continuous' && 'lower' in limits && 'upper' in limits ) {

					result += `lower="${ limits.lower }" upper="${ limits.upper }`;

				}

				result += '/>';

			}

			result += `${ indent1 }</joint>`;

			jointNodes.push( result );

		}

	}

}
