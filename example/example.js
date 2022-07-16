import { URDFExporter } from '../src/index.js';
import URDFLoader from 'urdf-loader';
import URDFViewer from 'urdf-loader/src/urdf-viewer-element.js';
import * as THREE from 'three';

customElements.define( 'urdf-viewer', URDFViewer );
const exporter = new URDFExporter();
const loader = new URDFLoader();
const el = document.querySelector( 'urdf-viewer' );

const url = 'https://raw.githubusercontent.com/gkjohnson/urdf-loaders/master/urdf/T12/urdf/T12.URDF';
loader.load( url, robot => {

	// time out so the meshes have had time to load
	// we should have an event on the parser for when all
	// meshes have loaded

	// because we're loading from a URDFParser data here, there will be a lot of
	// extra / unneeded nodes.
	setTimeout( () => {

		const exporter = new URDFExporter();
		const res = exporter.parse( robot );
		console.log(res);





		// const data = exporter.parse(robot, jointFunc, { collapse: true, createMeshCb });
		// el.loadingManager.setURLModifier(url => {

		//     if (/urdf$/i.test(url)) return URL.createObjectURL(new Blob([data.data]));

		//     const mesh = data.meshes
		//         .filter(m => new RegExp(`${ m.directory }${ m.name }\\.${ m.ext }$`).test(url))
		//         .pop();

		//     if (mesh != null) return URL.createObjectURL(new Blob([mesh.data]));

		//     const tex = data.textures
		//         .filter(t => new RegExp(`${ t.directory }${ t.name }\\.${ t.ext }$`).test(url))
		//         .pop();

		//     if (tex != null) return URL.createObjectURL(new Blob([tex.data]));

		//     return url;

		// });

		// el.urdf = 'exported.urdf';

		// const zip = new JSZip();
		// zip.file('T12.URDF', data.urdf);
		// data.meshes.forEach(m => zip.file(`${ m.directory }${ m.name }.${ m.ext }`, m.data));
		// data.textures.forEach(t => zip.file(`${ t.directory }${ t.name }.${ t.ext }`, t.data));

		// zip
		//     .generateAsync({ type: 'uint8array' })
		//     .then(zipdata => saveData(zipdata, 't12urdf.zip'));

	}, 3000 );

} );
