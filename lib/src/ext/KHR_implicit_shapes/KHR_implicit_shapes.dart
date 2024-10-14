library gltf.extensions.khr_implicit_shapes;

import 'package:gltf/src/base/gltf_property.dart';
import 'package:gltf/src/ext/extensions.dart';
import 'package:vector_math/vector_math.dart';

const Extension khrImplicitShapesExtension =
    Extension('KHR_implicit_shapes', <Type, ExtensionDescriptor>{
  Gltf: ExtensionDescriptor(KhrImplicitShapesGltf.fromMap),
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

const List<String> KHR_IMPLICIT_SHAPES_GLTF_MEMBERS = <String>[SHAPES];
const List<String> KHR_IMPLICIT_SHAPES_SHAPE_TYPES = <String>[
  SPHERE,
  BOX,
  CAPSULE,
  CYLINDER
];
const List<String> KHR_IMPLICIT_SHAPES_SHAPE_MEMBERS = <String>[
  TYPE,
  SPHERE,
  BOX,
  CAPSULE,
  CYLINDER
];
const List<String> KHR_IMPLICIT_SHAPES_SPHERE_MEMBERS = <String>[RADIUS];
const List<String> KHR_IMPLICIT_SHAPES_BOX_MEMBERS = <String>[SIZE];
const List<String> KHR_IMPLICIT_SHAPES_CAPSULE_MEMBERS = <String>[
  RADIUS_BOTTOM,
  RADIUS_TOP,
  HEIGHT
];
const List<String> KHR_IMPLICIT_SHAPES_CYLINDER_MEMBERS = <String>[
  RADIUS_BOTTOM,
  RADIUS_TOP,
  HEIGHT
];
const List<String> KHR_IMPLICIT_SHAPES_MESH_MEMBERS = <String>[
  MESH,
  WEIGHTS,
  USE_NODE_WEIGHTS,
  SKIN
];

//<todo.eoin I really want to be able to call the DataError (etc.) constructors
class ShapeError extends IssueType {
  static final compoundShapeError = ShapeError._(
      'KHR_IMPLICIT_SHAPES_COMPOUND_SHAPE',
      (args) => 'Shape contains multiple different geometries');
  static final degenerateGeometry = ShapeError._(
      'KHR_IMPLICIT_SHAPES_DEGENERATE_SHAPE',
      (args) => 'Shape geometry is degenerate',
      Severity.Information);
  static final duplicateWeights = ShapeError._(
      'KHR_IMPLICIT_SHAPES_DUPLICATE_WEIGHTS',
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

class KhrImplicitShapesGltf extends GltfProperty {
  final SafeList<KhrImplicitShapesShape> shapes;

  KhrImplicitShapesGltf._(
      this.shapes, Map<String, Object> extensions, Object extras)
      : super(extensions, extras);

  static KhrImplicitShapesGltf fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, KHR_IMPLICIT_SHAPES_GLTF_MEMBERS, context);
    }

    SafeList<KhrImplicitShapesShape> shapes;
    final shapeMaps = getMapList(map, SHAPES, context);
    if (shapeMaps != null) {
      shapes = SafeList<KhrImplicitShapesShape>(shapeMaps.length, SHAPES);
      context.path.add(SHAPES);
      for (var i = 0; i < shapeMaps.length; i++) {
        final shapeMap = shapeMaps[i];
        context.path.add(i.toString());
        shapes[i] = KhrImplicitShapesShape.fromMap(shapeMap, context);
        context.path.removeLast();
      }
      context.path.removeLast();
    } else {
      shapes = SafeList<KhrImplicitShapesShape>.empty(SHAPES);
    }

    return KhrImplicitShapesGltf._(
        shapes,
        getExtensions(map, KhrImplicitShapesGltf, context),
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

class KhrImplicitShapesShape extends GltfChildOfRootProperty {
  final String type;
  KhrImplicitShapesShapeGeometry geom;

  KhrImplicitShapesShape._(this.type, this.geom, String name,
      Map<String, Object> extensions, Object extras)
      : super(name, extensions, extras);

  static KhrImplicitShapesShape fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, KHR_IMPLICIT_SHAPES_SHAPE_MEMBERS, context);
    }

    final type = getString(map, TYPE, context,
        list: KHR_IMPLICIT_SHAPES_SHAPE_TYPES, req: true);

    const typeFns = [
      KhrImplicitShapesShapeSphere.fromMap,
      KhrImplicitShapesShapeBox.fromMap,
      KhrImplicitShapesShapeCapsule.fromMap,
      KhrImplicitShapesShapeCylinder.fromMap
    ];
    assert(KHR_IMPLICIT_SHAPES_SHAPE_TYPES.length == typeFns.length);

    KhrImplicitShapesShapeGeometry geometry;
    for (var i = 0; i < typeFns.length; i++) {
      if (map.containsKey(KHR_IMPLICIT_SHAPES_SHAPE_TYPES[i])) {
        if (geometry != null) {
          context.addIssue(ShapeError.compoundShapeError,
              name: KHR_IMPLICIT_SHAPES_SHAPE_TYPES[i]);
        }
        geometry = getObjectFromInnerMap(
            map, KHR_IMPLICIT_SHAPES_SHAPE_TYPES[i], context, typeFns[i],
            req: false);
      }
    }

    final extensions = getExtensions(map, KhrImplicitShapesShape, context);

    return KhrImplicitShapesShape._(type, geometry, getName(map, context),
        extensions, getExtras(map, context));
  }

  @override
  void link(Gltf gltf, Context context) {
    if (context.validate && geom != null) {
      geom.link(gltf, context);
    }
  }
}

class KhrImplicitShapesShapeGeometry extends GltfProperty {
  KhrImplicitShapesShapeGeometry(Map<String, Object> extensions, Object extras)
      : super(extensions, extras);
}

class KhrImplicitShapesShapeSphere extends KhrImplicitShapesShapeGeometry {
  final double radius;

  KhrImplicitShapesShapeSphere._(
      this.radius, Map<String, Object> extensions, Object extras)
      : super(extensions, extras);

  static KhrImplicitShapesShapeGeometry fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, KHR_IMPLICIT_SHAPES_SPHERE_MEMBERS, context);
    }

    final radius = getFloat(map, RADIUS, context, min: 0, def: 0.5);
    if (context.validate && radius == 0) {
      context.addIssue(ShapeError.degenerateGeometry, name: RADIUS);
    }
    return KhrImplicitShapesShapeSphere._(
        radius,
        getExtensions(map, KhrImplicitShapesShapeSphere, context),
        getExtras(map, context));
  }
}

class KhrImplicitShapesShapeBox extends KhrImplicitShapesShapeGeometry {
  final Vector3 size;

  KhrImplicitShapesShapeBox._(
      this.size, Map<String, Object> extensions, Object extras)
      : super(extensions, extras);

  static KhrImplicitShapesShapeGeometry fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, KHR_IMPLICIT_SHAPES_BOX_MEMBERS, context);
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

    return KhrImplicitShapesShapeBox._(
        _listToVec3(sizeList),
        getExtensions(map, KhrImplicitShapesShapeBox, context),
        getExtras(map, context));
  }

  static Vector3 _listToVec3(List<double> l) {
    if (l != null) {
      return Vector3(l[0], l[1], l[2]);
    }
    return null;
  }
}

class KhrImplicitShapesShapeCapsule extends KhrImplicitShapesShapeGeometry {
  final double radiusBottom;
  final double radiusTop;
  final double height;

  KhrImplicitShapesShapeCapsule._(this.radiusBottom, this.radiusTop,
      this.height, Map<String, Object> extensions, Object extras)
      : super(extensions, extras);

  static KhrImplicitShapesShapeGeometry fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, KHR_IMPLICIT_SHAPES_CAPSULE_MEMBERS, context);
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

    return KhrImplicitShapesShapeCapsule._(
        radiusBottom,
        radiusTop,
        height,
        getExtensions(map, KhrImplicitShapesShapeCapsule, context),
        getExtras(map, context));
  }
}

class KhrImplicitShapesShapeCylinder extends KhrImplicitShapesShapeGeometry {
  final double radiusBottom;
  final double radiusTop;
  final double height;

  KhrImplicitShapesShapeCylinder._(this.radiusBottom, this.radiusTop,
      this.height, Map<String, Object> extensions, Object extras)
      : super(extensions, extras);

  static KhrImplicitShapesShapeGeometry fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, KHR_IMPLICIT_SHAPES_CYLINDER_MEMBERS, context);
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
    return KhrImplicitShapesShapeCylinder._(
        radiusBottom,
        radiusTop,
        height,
        getExtensions(map, KhrImplicitShapesShapeCylinder, context),
        getExtras(map, context));
  }
}
