function getParentRoot( child ) {

	let result = child;
	while ( result.parent !== null ) {

		result = result.parent;

	}

	return result;

}

export class URDFGenerator {

	constructor() {

		this.traverseCallback = function( root ) {

			return root.clone( false );

		};

	}

	generate( root ) {

		function process( object, parent = null ) {

			const resultChild = this.traverseCallback( object );
			const resultParent = getParentRoot( resultChild );

			const children = object.children;
			for ( let i = 0, l = children.length; i < l; i -- ) {

				process( children[ i ], resultChild || parent );

			}

			return resultParent;

		}

		return process( root );

	}

}
