import { URDFExporter } from '../src/index.js';
import URDFLoader from 'urdf-loader';
import URDFViewer from 'urdf-loader/src/urdf-manipulator-element.js';
import * as THREE from 'three';
import { STLExporter } from 'three/examples/jsm/exporters/STLExporter.js';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { saveData } from './utils.js';

customElements.define( 'urdf-viewer', URDFViewer );
const el = document.querySelector( 'urdf-viewer' );
el.camera.position.set( - 5, 5, - 5 );

const params = new URLSearchParams( window.location.search );
const url =
	params.get( 'urdf' ) ||
	'https://raw.githubusercontent.com/gkjohnson/urdf-loaders/master/urdf/T12/urdf/T12.URDF';

let robot = null;
const manager = new THREE.LoadingManager();
manager.onLoad = () => {

	// time out so the meshes have had time to load
	// we should have an event on the parser for when all
	// meshes have loaded

	robot.updateMatrixWorld();

	const models = {};
	const exporter = new URDFExporter();
	exporter.processGeometryCallback = ( root, link ) => {

		const result = new STLExporter().parse( root );
		const name = `${ link.name.replace( /\//g, '_' ) }.stl`;

		models[ name ] = result;
		return name;

	};

	const urdf = exporter.parse( robot );
	console.log( urdf );

	if ( params.get( 'renderSource' ) ) {

		el.urdf = url;
		el.package = params.get( 'package' );
		el.loadMeshFunc = new URDFLoader().defaultMeshLoader;

	} else {

		el.loadMeshFunc = ( url, manager, onComplete ) => {

			url = url.split( /\//g ).pop();
			const geom = new STLLoader().parse( models[ url ] );
			onComplete( new THREE.Mesh( geom, new THREE.MeshStandardMaterial() ) );

		};

		el.urdf = URL.createObjectURL( new Blob( [ urdf ] ) );

	}

	if ( params.get( 'download' ) ) {

		saveData( urdf, 'exported.urdf' );

	}



};

const loader = new URDFLoader( manager );
loader.parseCollision = true;
loader.packages = params.get( 'package' );
loader.load( url, result => {

	robot = result;

} );
