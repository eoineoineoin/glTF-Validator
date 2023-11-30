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
const String CONVEX = 'convex';
const String TRIMESH = 'trimesh';
const String RADIUS = 'radius';
const String RADIUS_BOTTOM = 'radiusBottom';
const String RADIUS_TOP = 'radiusTop';
const String SIZE = 'size';
const String HEIGHT = 'height';
const String MESH = 'mesh';

const List<String> KHR_COLLISION_SHAPES_GLTF_MEMBERS = <String>[SHAPES];
const List<String> KHR_COLLISION_SHAPES_SHAPE_TYPES = <String>[
  SPHERE,
  BOX,
  CAPSULE,
  CYLINDER,
  CONVEX,
  TRIMESH
];
const List<String> KHR_COLLISION_SHAPES_SHAPE_MEMBERS = <String>[
  TYPE,
  SPHERE,
  BOX,
  CAPSULE,
  CYLINDER,
  CONVEX,
  TRIMESH
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
const List<String> KHR_COLLISION_SHAPES_CONVEX_MEMBERS = <String>[MESH];
const List<String> KHR_COLLISION_SHAPES_TRIMESH_MEMBERS = <String>[MESH];

//<todo.eoin I really want to be able to call the DataError (etc.) constructors
class ShapeError extends IssueType {
  static final compoundShapeError = ShapeError._(
      'KHR_COLLISION_SHAPES_COMPOUND_SHAPE',
      (args) => 'Shape contains multiple different geometries');
  static final degenerateGeometry = ShapeError._(
      'KHR_COLLISION_SHAPES_DEGENERATE_SHAPE',
      (args) => 'Shape geometry is degenerate',
      Severity.Information);
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
      KhrCollisionShapesShapeConvex.fromMap,
      KhrCollisionShapesShapeTrimesh.fromMap
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

class KhrCollisionShapesShapeConvex extends KhrCollisionShapesShapeGeometry {
  final int _meshIndex;
  Mesh _mesh;

  KhrCollisionShapesShapeConvex._(
      this._meshIndex, Map<String, Object> extensions, Object extras)
      : super(extensions, extras);

  static KhrCollisionShapesShapeGeometry fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, KHR_COLLISION_SHAPES_CONVEX_MEMBERS, context);
    }

    final mesh = getIndex(map, MESH, context, req: true);
    return KhrCollisionShapesShapeConvex._(
        mesh,
        getExtensions(map, KhrCollisionShapesShapeConvex, context),
        getExtras(map, context));
  }

  @override
  void link(Gltf gltf, Context context) {
    if (context.validate) {
      _mesh = gltf.meshes[_meshIndex];
      if (_mesh == null) {
        context.addIssue(LinkError.unresolvedReference,
            name: MESH, args: [_meshIndex]);
      } else {
        _mesh.markAsUsed();
      }
    }
  }

  Mesh get mesh => _mesh;
}

class KhrCollisionShapesShapeTrimesh extends KhrCollisionShapesShapeGeometry {
  final int _meshIndex;
  Mesh _mesh;

  KhrCollisionShapesShapeTrimesh._(
      this._meshIndex, Map<String, Object> extensions, Object extras)
      : super(extensions, extras);

  static KhrCollisionShapesShapeGeometry fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, KHR_COLLISION_SHAPES_TRIMESH_MEMBERS, context);
    }

    final mesh = getIndex(map, MESH, context, req: true);
    return KhrCollisionShapesShapeTrimesh._(
        mesh,
        getExtensions(map, KhrCollisionShapesShapeCylinder, context),
        getExtras(map, context));
  }

  @override
  void link(Gltf gltf, Context context) {
    if (context.validate) {
      _mesh = gltf.meshes[_meshIndex];
      if (_mesh == null) {
        context.addIssue(LinkError.unresolvedReference,
            name: MESH, args: [_meshIndex]);
      } else {
        _mesh.markAsUsed();
      }
    }
  }

  Mesh get mesh => _mesh;
}
