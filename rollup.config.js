const path = require( 'path' );
const inputPath = path.join( __dirname, './src/index.js' );

export default [ {

	input: inputPath,
	treeshake: false,
	external: p => /^three/.test( p ),

	output: {

		name: 'URDFExporter',
		extend: true,
		format: 'umd',
		file: path.join( __dirname, './build/index.umd.cjs' ),
		sourcemap: true,

		globals: path => /^three/.test( path ) ? 'THREE' : null,

	},

}, {

	input: inputPath,
	treeshake: false,
	external: p => /^three/.test( p ),

	output: {

		name: 'URDFExporter',
		extend: true,
		format: 'es',
		file: path.join( __dirname, './build/index.module.js' ),
		sourcemap: true,

		globals: path => /^three/.test( path ) ? 'THREE' : null,

	},

} ];
