// https://stackoverflow.com/questions/19327749/javascript-blob-filename-without-link
const saveData = ( function saveData() {

	const a = document.createElement( 'a' );
	document.body.appendChild( a );
	a.style = 'display: none';
	return function ( data, fileName ) {

		const blob = new Blob( [ data ], { type: 'octet/stream' } );
		const url = window.URL.createObjectURL( blob );
		a.href = url;
		a.download = fileName;
		a.click();
		a.remove();
		window.URL.revokeObjectURL( url );

	};

}() );

export { saveData };
