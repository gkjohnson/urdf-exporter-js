function getParentRoot( child ) {

	let result = child;
	while ( result.parent !== null ) {

		result = result.parent;

	}

	return result;

}

export class URDFGenerator {

	constructor() {

		this.generateCallback = function ( root ) {

			return root.clone( false );

		};

		this.postprocessCallback = function ( root ) {

		};

	}

	generate( root ) {

		const process = ( object, parent = null ) => {

			const resultChild = this.generateCallback( object );
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
		return result;

	}

}
