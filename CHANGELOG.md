# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/en/1.0.0/)
and this project adheres to [Semantic Versioning](http://semver.org/spec/v2.0.0.html).

## Unreleased
### Changed
- Moved `robotName` from an option in the `parse` function to an option.
- URDF contents are now in the `data` field instead of the `urdf` field.

### Added
- `pathPrefix` option.
- Ability to return `null` from `meshFunc` to indicate no visual node.
- `isLeaf` value from jointFunc to indicate that traversal should stop.

### Removed
- `packagePrefix` option.

### Fixed
- The euler order at the end of a rotation.

## [0.1.3] - 2018-06-25
### Added
- Provide directories for meshes and texture output
