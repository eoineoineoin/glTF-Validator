library gltf.extensions.khr_collision_shapes;

import 'package:gltf/src/base/gltf_property.dart';
import 'package:gltf/src/ext/extensions.dart';
import 'package:vector_math/vector_math.dart';

const Extension khrCollisionShapesExtension =
    Extension('KHR_collision_shapes', <Type, ExtensionDescriptor>{
  Gltf: ExtensionDescriptor(KhrCollisionShapesGltf.fromMap),
});

const String SHAPES = 'shapes';
const String SPHERE = 'sphere';
const String BOX = 'box';
const String CAPSULE = 'capsule';
const String CYLINDER = 'cylinder';
const String RADIUS = 'radius';
const String RADIUS_BOTTOM = 'radiusBottom';
const String RADIUS_TOP = 'radiusTop';
const String SIZE = 'size';
const String HEIGHT = 'height';
const String MESH = 'mesh';
const String WEIGHTS = 'weights';
const String USE_NODE_WEIGHTS = 'useNodeWeights';
const String SKIN = 'skin';

const List<String> KHR_COLLISION_SHAPES_GLTF_MEMBERS = <String>[SHAPES];
const List<String> KHR_COLLISION_SHAPES_SHAPE_TYPES = <String>[
  SPHERE,
  BOX,
  CAPSULE,
  CYLINDER,
  MESH
];
const List<String> KHR_COLLISION_SHAPES_SHAPE_MEMBERS = <String>[
  TYPE,
  SPHERE,
  BOX,
  CAPSULE,
  CYLINDER,
  MESH
];
const List<String> KHR_COLLISION_SHAPES_SPHERE_MEMBERS = <String>[RADIUS];
const List<String> KHR_COLLISION_SHAPES_BOX_MEMBERS = <String>[SIZE];
const List<String> KHR_COLLISION_SHAPES_CAPSULE_MEMBERS = <String>[
  RADIUS_BOTTOM,
  RADIUS_TOP,
  HEIGHT
];
const List<String> KHR_COLLISION_SHAPES_CYLINDER_MEMBERS = <String>[
  RADIUS_BOTTOM,
  RADIUS_TOP,
  HEIGHT
];
const List<String> KHR_COLLISION_SHAPES_MESH_MEMBERS = <String>[
  MESH,
  WEIGHTS,
  USE_NODE_WEIGHTS,
  SKIN
];

//<todo.eoin I really want to be able to call the DataError (etc.) constructors
class ShapeError extends IssueType {
  static final compoundShapeError = ShapeError._(
      'KHR_COLLISION_SHAPES_COMPOUND_SHAPE',
      (args) => 'Shape contains multiple different geometries');
  static final degenerateGeometry = ShapeError._(
      'KHR_COLLISION_SHAPES_DEGENERATE_SHAPE',
      (args) => 'Shape geometry is degenerate',
      Severity.Information);
  static final duplicateWeights = ShapeError._(
      'KHR_COLLISION_SHAPES_DUPLICATE_WEIGHTS',
      (args) => 'Only one of "weights" and "useNodeWeights" should be provided',
      Severity.Error);
  static final shapeWeightsInvalid = ShapeError._(
      'SHAPE_WEIGHTS_INVALID',
      (args) => 'The length of weights array (${args[0]}) does not match '
          'the number of morph targets (${args[1] ?? 0}).');
  ShapeError._(String type, ErrorFunction message,
      [Severity severity = Severity.Error])
      : super(type, message, severity);
}

class KhrCollisionShapesGltf extends GltfProperty {
  final SafeList<KhrCollisionShapesShape> shapes;

  KhrCollisionShapesGltf._(
      this.shapes, Map<String, Object> extensions, Object extras)
      : super(extensions, extras);

  static KhrCollisionShapesGltf fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, KHR_COLLISION_SHAPES_GLTF_MEMBERS, context);
    }

    SafeList<KhrCollisionShapesShape> shapes;
    final shapeMaps = getMapList(map, SHAPES, context);
    if (shapeMaps != null) {
      shapes = SafeList<KhrCollisionShapesShape>(shapeMaps.length, SHAPES);
      context.path.add(SHAPES);
      for (var i = 0; i < shapeMaps.length; i++) {
        final shapeMap = shapeMaps[i];
        context.path.add(i.toString());
        shapes[i] = KhrCollisionShapesShape.fromMap(shapeMap, context);
        context.path.removeLast();
      }
      context.path.removeLast();
    } else {
      shapes = SafeList<KhrCollisionShapesShape>.empty(SHAPES);
    }

    return KhrCollisionShapesGltf._(
        shapes,
        getExtensions(map, KhrCollisionShapesGltf, context),
        getExtras(map, context));
  }

  @override
  void link(Gltf gltf, Context context) {
    context.path.add(SHAPES);
    context.extensionCollections[shapes] = context.path.toList(growable: false);
    shapes.forEachWithIndices((i, shape) {
      context.path.add(i.toString());
      shape.link(gltf, context);
      context.path.removeLast();
    });
    context.path.removeLast();
  }
}

class KhrCollisionShapesShape extends GltfChildOfRootProperty {
  final String type;
  KhrCollisionShapesShapeGeometry geom;

  KhrCollisionShapesShape._(this.type, this.geom, String name,
      Map<String, Object> extensions, Object extras)
      : super(name, extensions, extras);

  static KhrCollisionShapesShape fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, KHR_COLLISION_SHAPES_SHAPE_MEMBERS, context);
    }

    final type = getString(map, TYPE, context,
        list: KHR_COLLISION_SHAPES_SHAPE_TYPES, req: true);

    const typeFns = [
      KhrCollisionShapesShapeSphere.fromMap,
      KhrCollisionShapesShapeBox.fromMap,
      KhrCollisionShapesShapeCapsule.fromMap,
      KhrCollisionShapesShapeCylinder.fromMap,
      KhrCollisionShapesShapeMesh.fromMap
    ];
    assert(KHR_COLLISION_SHAPES_SHAPE_TYPES.length == typeFns.length);

    KhrCollisionShapesShapeGeometry geometry;
    for (var i = 0; i < typeFns.length; i++) {
      if (map.containsKey(KHR_COLLISION_SHAPES_SHAPE_TYPES[i])) {
        if (geometry != null) {
          context.addIssue(ShapeError.compoundShapeError,
              name: KHR_COLLISION_SHAPES_SHAPE_TYPES[i]);
        }
        geometry = getObjectFromInnerMap(
            map, KHR_COLLISION_SHAPES_SHAPE_TYPES[i], context, typeFns[i],
            req: false);
      }
    }

    final extensions = getExtensions(map, KhrCollisionShapesShape, context);

    return KhrCollisionShapesShape._(type, geometry, getName(map, context),
        extensions, getExtras(map, context));
  }

  @override
  void link(Gltf gltf, Context context) {
    if (context.validate && geom != null) {
      geom.link(gltf, context);
    }
  }
}

class KhrCollisionShapesShapeGeometry extends GltfProperty {
  KhrCollisionShapesShapeGeometry(Map<String, Object> extensions, Object extras)
      : super(extensions, extras);
}

class KhrCollisionShapesShapeSphere extends KhrCollisionShapesShapeGeometry {
  final double radius;

  KhrCollisionShapesShapeSphere._(
      this.radius, Map<String, Object> extensions, Object extras)
      : super(extensions, extras);

  static KhrCollisionShapesShapeGeometry fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, KHR_COLLISION_SHAPES_SPHERE_MEMBERS, context);
    }

    final radius = getFloat(map, RADIUS, context, min: 0, def: 0.5);
    if (context.validate && radius == 0) {
      context.addIssue(ShapeError.degenerateGeometry, name: RADIUS);
    }
    return KhrCollisionShapesShapeSphere._(
        radius,
        getExtensions(map, KhrCollisionShapesShapeSphere, context),
        getExtras(map, context));
  }
}

class KhrCollisionShapesShapeBox extends KhrCollisionShapesShapeGeometry {
  final Vector3 size;

  KhrCollisionShapesShapeBox._(
      this.size, Map<String, Object> extensions, Object extras)
      : super(extensions, extras);

  static KhrCollisionShapesShapeGeometry fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, KHR_COLLISION_SHAPES_BOX_MEMBERS, context);
    }

    final sizeList = getFloatList(map, SIZE, context,
        min: 0, lengthsList: const [3], def: const [1.0, 1.0, 1.0]);

    if (context.validate && sizeList != null) {
      context.path.add(SIZE);
      for (var i = 0; i < 3; i++) {
        context.path.add(i.toString());
        if (sizeList[i] == 0) {
          context.addIssue(ShapeError.degenerateGeometry);
        }
        context.path.removeLast();
      }
      context.path.removeLast();
    }

    return KhrCollisionShapesShapeBox._(
        _listToVec3(sizeList),
        getExtensions(map, KhrCollisionShapesShapeBox, context),
        getExtras(map, context));
  }

  static Vector3 _listToVec3(List<double> l) {
    if (l != null) {
      return Vector3(l[0], l[1], l[2]);
    }
    return null;
  }
}

class KhrCollisionShapesShapeCapsule extends KhrCollisionShapesShapeGeometry {
  final double radiusBottom;
  final double radiusTop;
  final double height;

  KhrCollisionShapesShapeCapsule._(this.radiusBottom, this.radiusTop,
      this.height, Map<String, Object> extensions, Object extras)
      : super(extensions, extras);

  static KhrCollisionShapesShapeGeometry fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, KHR_COLLISION_SHAPES_CAPSULE_MEMBERS, context);
    }

    final radiusBottom =
        getFloat(map, RADIUS_BOTTOM, context, min: 0, def: 0.25);
    final radiusTop = getFloat(map, RADIUS_TOP, context, min: 0, def: 0.25);
    final height = getFloat(map, HEIGHT, context, min: 0, def: 0.5);

    if (context.validate) {
      if (radiusTop == 0 && radiusBottom == 0) {
        context.addIssue(ShapeError.degenerateGeometry);
      }
      if (height == 0) {
        context.addIssue(ShapeError.degenerateGeometry, name: HEIGHT);
      }

      if (height + radiusBottom + radiusTop < unitSumThresholdStep) {
        context.addIssue(ShapeError.degenerateGeometry);
      }
    }

    return KhrCollisionShapesShapeCapsule._(
        radiusBottom,
        radiusTop,
        height,
        getExtensions(map, KhrCollisionShapesShapeCapsule, context),
        getExtras(map, context));
  }
}

class KhrCollisionShapesShapeCylinder extends KhrCollisionShapesShapeGeometry {
  final double radiusBottom;
  final double radiusTop;
  final double height;

  KhrCollisionShapesShapeCylinder._(this.radiusBottom, this.radiusTop,
      this.height, Map<String, Object> extensions, Object extras)
      : super(extensions, extras);

  static KhrCollisionShapesShapeGeometry fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, KHR_COLLISION_SHAPES_CYLINDER_MEMBERS, context);
    }

    final radiusBottom =
        getFloat(map, RADIUS_BOTTOM, context, min: 0, def: 0.25);
    final radiusTop = getFloat(map, RADIUS_TOP, context, min: 0, def: 0.25);
    final height = getFloat(map, HEIGHT, context, min: 0, def: 0.5);

    if (context.validate) {
      if (radiusBottom == 0 && radiusTop == 0) {
        context.addIssue(ShapeError.degenerateGeometry);
      }
      if (height == 0) {
        context.addIssue(ShapeError.degenerateGeometry, name: HEIGHT);
      }
    }
    return KhrCollisionShapesShapeCylinder._(
        radiusBottom,
        radiusTop,
        height,
        getExtensions(map, KhrCollisionShapesShapeCylinder, context),
        getExtras(map, context));
  }
}

class KhrCollisionShapesShapeMesh extends KhrCollisionShapesShapeGeometry {
  final int _meshIndex;
  final int _skinIndex;
  final List<double> weights;
  final bool useNodeWeights;
  Mesh _mesh;
  Skin _skin;

  KhrCollisionShapesShapeMesh._(this._meshIndex, this._skinIndex, this.weights,
      this.useNodeWeights, Map<String, Object> extensions, Object extras)
      : super(extensions, extras);

  static KhrCollisionShapesShapeGeometry fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, KHR_COLLISION_SHAPES_MESH_MEMBERS, context);
    }

    final mesh = getIndex(map, MESH, context, req: true);
    final skinIndex = getIndex(map, SKIN, context, req: false);
    final weightsList = getFloatList(map, WEIGHTS, context, req: false);
    // Would like if getBool() had a 'req' parameter, but when the
    // key does not exist, the return value matches our default
    final useNodeWeights = getBool(map, USE_NODE_WEIGHTS, context);

    if (weightsList != null && useNodeWeights) {
      context.addIssue(ShapeError.duplicateWeights);
    }

    return KhrCollisionShapesShapeMesh._(
        mesh,
        skinIndex,
        weightsList,
        useNodeWeights,
        getExtensions(map, KhrCollisionShapesShapeMesh, context),
        getExtras(map, context));
  }

  @override
  void link(Gltf gltf, Context context) {
    _mesh = gltf.meshes[_meshIndex];
    _skin = gltf.skins[_skinIndex];

    if (context.validate) {
      if (_mesh == null) {
        context.addIssue(LinkError.unresolvedReference,
            name: MESH, args: [_meshIndex]);
      } else {
        _mesh.markAsUsed();

        if (weights == null && _mesh.weights != null) {
          _mesh.markWeightsAsUsed();
        }

        if (weights != null &&
            _mesh.primitives[0].targets?.length != weights.length) {
          context.addIssue(ShapeError.shapeWeightsInvalid,
              name: WEIGHTS,
              args: [weights.length, _mesh.primitives[0].targets?.length]);
        }
      }

      if (_skinIndex != -1) {
        if (_skin == null) {
          context.addIssue(LinkError.unresolvedReference,
              name: SKIN, args: [_skinIndex]);
        } else {
          _skin.markAsUsed();
        }
      }
    }
  }

  Mesh get mesh => _mesh;
  Skin get skin => _skin;
}
