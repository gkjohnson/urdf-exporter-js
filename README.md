# urdf-exporter-js

[![npm version](https://img.shields.io/npm/v/urdf-exporter.svg?style=flat-square)](https://www.npmjs.com/package/urdf-exporter)
[![lgtm code quality](https://img.shields.io/lgtm/grade/javascript/g/gkjohnson/urdf-exporter-js.svg?style=flat-square&label=code-quality)](https://lgtm.com/projects/g/gkjohnson/urdf-exporter-js/)
[![build](https://img.shields.io/github/workflow/status/gkjohnson/urdf-exporter-js/Node.js%20CI?style=flat-square&label=build)](https://github.com/gkjohnson/urdf-exporter-js/actions)
[![github](https://flat.badgen.net/badge/icon/github?icon=github&label)](https://github.com/gkjohnson/urdf-exporter-js/)
[![twitter](https://flat.badgen.net/twitter/follow/garrettkjohnson)](https://twitter.com/garrettkjohnson)
[![sponsors](https://img.shields.io/github/sponsors/gkjohnson?style=flat-square&color=1da1f2)](https://github.com/sponsors/gkjohnson/)

Utility for exporting a three.js object hierarchy as a URDF.

# Use

```js
import { URDFExporter } from 'urdf-exporter';
import { STLExporter } from 'three/examples/jsm/exporters/STLExporter.js';

let urdfModel;
// ... create a model using the URDF classes for export

const models = {};
const exporter = new URDFExporter();
exporter.processGeometryCallback = ( model, link ) => {

	const name = `${ link.name }_mesh.stl`;
	models[ name ] = new STLExporter().parse( model );
	return name;

};

urdfModel.updateMatrixWorld();
const urdf = exporter.parse( urdfModel );

// ... urdf content ready!

```


# API

## URDFGenerator

Utility class to enable convenient conversion from three.js objects to URDF classes for export.

### generateCallback

```js
generateCallback : ( object : Object3D ) => Object3D
```

Callback used for generating URDF class equivalents. The function is expected to return a cloned version of the provided mesh or a URDF class describing the analogous object. The returned object must be the child object that will have children added to it. The generator will then find the root parent to add to the previously processed object.

For example a custom three.js joint type could converted into a joint and link connection which can be run through the `URDFExporter`.

### postprocessCallback

```js
postprocessCallback : ( object : URDFRobot ) => void
```

A function that takes the generated URDF result to enable fixups and other types of postprocessing that might be needed.

## URDFExporter

### .indent

```js
indent = '\t' : string
```

The set of indentation characters to use.

### .processGeometryCallback

```js
processGeometryCallback : ( node : Object3D, link : URDFLink ) => string
```

The callback for to use when processing geometry. Geometry must be processed and cached in order to be exported. A file path is returned from the function.

### .parse

```js
parse( root : URDFRobot ) : string
```

Parses the object into a urdf file. Returns the URDF contents as a string. The hierarchy matrix world must be updated before calling this function.

## URDFLimit

Class containing values to export for joint limits.

### .upper

```js
upper = 0 : Number
```

### .lower

```js
lower = 0 : Number
```

### .velocity

```js
velocity = 0 : Number
```

### .effort

```js
effort = 0 : Number
```

## URDFInertialFrame

Class containing values for the link inertial frame.

### .position

```js
position : Vector3
```

### .rotation

```js
rotation : Euler
```

### .mass

```js
mass = 0 : Number
```

### inertia

```js
inertial : Matrix3
```

The upper triangular matrix is used to define the `xx`, `yy`, `zz`, `xy`, `yz`, and `xz` fields.

## URDFLink

_extends THREE.Object3D_

When this field is encountered a new link is created in the URDF file.

### .inertial

```js
inertial : URDFInertialFrame
```

## URDFJoint

_extends THREE.Object3D_

When this field is encountered a new joint is created in the URDF file.

### .jointType

```js
jointType = 'fixed' : string
```

### .axis

```js
axis : Vector3
```

### .limit

```js
limit : URDFLimit
```

## URDFRobot

_extends URDFLink_

A class describing the root of the URDF Robot.

### .robotName

```js
robotName = '' : string
```

## URDFVisual

_extends THREE.Object3D_

## URDFCollider

_extends THREE.Object3D_
