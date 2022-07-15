# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/en/1.0.0/)
and this project adheres to [Semantic Versioning](http://semver.org/spec/v2.0.0.html).

## [0.4.0] - unreleased
### Changed
- Rewrite of exporter to make results more reliable and consistent. Exported URDF classes are now required for exporting a model. Convert any target export into the model that uses the URDF classes.

## [0.3.1] - 2018-10-30
### Changed
- Moved URDFExporter.js file to `src` folder.
- Added UMD variant of URDFExporter to `umd` folder.
- Added an `onComplete` callback to the `parse` function

## [0.2.0] - 2018-08-23
### Changed
- Moved `robotName` from an option in the `parse` function to an option.
- URDF contents are now in the `data` field instead of the `urdf` field.

### Added
- `pathPrefix` option.
- Ability to return `null` from `meshFunc` to indicate no visual node.
- `isLeaf` value from jointFunc to indicate that traversal should stop.
- Added return value from the mesh creation function to define material attributes including `color`, `opacity`, and `texture`.

### Removed
- `packagePrefix` option.
- `includeMaterials` field from createMeshCb function.

### Fixed
- The euler order at the end of a rotation.

## [0.1.3] - 2018-06-25
### Added
- Provide directories for meshes and texture output
