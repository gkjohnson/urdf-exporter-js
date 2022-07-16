// https://stackoverflow.com/questions/19327749/javascript-blob-filename-without-link
var saveData = ( function saveData() {

	var a = document.createElement( 'a' );
	document.body.appendChild( a );
	a.style = 'display: none';
	return function ( data, fileName ) {

		var blob = new Blob( [ data ], { type: 'octet/stream' } ),
			url = window.URL.createObjectURL( blob );
		a.href = url;
		a.download = fileName;
		a.click();
		window.URL.revokeObjectURL( url );

	};

}() );

export { saveData };
