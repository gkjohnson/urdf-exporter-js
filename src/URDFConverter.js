function getParentRoot( child ) {

	let result = child;
	while ( result.parent !== null ) {

		result = result.parent;

	}

	return result;

}

export class URDFConverter {

	constructor() {

		this.generateCallback = function ( root ) {

			return root.clone( false );

		};

		this.postprocessCallback = function ( root ) {

		};

	}

	generate( root ) {

		const process = ( object, parent = null ) => {

			if ( object.visible === false ) {

				return null;

			}

			const resultChild = this.generateCallback( object );
			if ( resultChild === null ) {

				return null;

			}

			const resultParent = getParentRoot( resultChild );
			if ( parent !== null ) {

				parent.add( resultParent );

			}

			const children = object.children;
			for ( let i = 0, l = children.length; i < l; i ++ ) {

				process( children[ i ], resultChild || parent );

			}

			return resultParent;

		};

		const result = process( root );
		this.postprocessCallback( root );

		result.updateMatrixWorld( true );
		return result;

	}

}
