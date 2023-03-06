library gltf.extensions.msft_collision_primitives;

import 'package:gltf/src/base/gltf_property.dart';
import 'package:gltf/src/ext/extensions.dart';
import 'package:vector_math/vector_math.dart';

const Extension msftCollisionPrimitivesExtension =
    Extension('MSFT_collision_primitives', <Type, ExtensionDescriptor>{
  Gltf: ExtensionDescriptor(MsftCollisionPrimitivesGltf.fromMap),
});

const String COLLIDERS = 'colliders';
const String COLLISION_SYSTEMS = 'collisionSystems';
const String COLLIDE_WITH_SYSTEMS = 'collideWithSystems';
const String NOT_COLLIDE_WITH_SYSTEMS = 'notCollideWithSystems';
const String SPHERE = 'sphere';
const String BOX = 'box';
const String CAPSULE = 'capsule';
const String CYLINDER = 'cylinder';
const String CONVEX = 'convex';
const String TRIMESH = 'trimesh';
const String RADIUS = 'radius';
const String SIZE = 'size';
const String HEIGHT = 'height';
const String MESH = 'mesh';

const List<String> MSFT_COLLISION_PRIMITIVES_GLTF_MEMBERS = <String>[COLLIDERS];
const List<String> MSFT_COLLISION_PRIMITIVES_COLLIDER_MEMBERS = <String>[
  COLLISION_SYSTEMS,
  COLLIDE_WITH_SYSTEMS,
  NOT_COLLIDE_WITH_SYSTEMS,
  SPHERE,
  BOX,
  CAPSULE,
  CYLINDER,
  CONVEX,
  TRIMESH
];
const List<String> MSFT_COLLISION_PRIMITIVES_SPHERE_MEMBERS = <String>[RADIUS];
const List<String> MSFT_COLLISION_PRIMITIVES_BOX_MEMBERS = <String>[SIZE];
const List<String> MSFT_COLLISION_PRIMITIVES_CAPSULE_MEMBERS = <String>[
  RADIUS,
  HEIGHT
];
const List<String> MSFT_COLLISION_PRIMITIVES_CYLINDER_MEMBERS = <String>[
  RADIUS,
  HEIGHT
];
const List<String> MSFT_COLLISION_PRIMITIVES_CONVEX_MEMBERS = <String>[MESH];
const List<String> MSFT_COLLISION_PRIMITIVES_TRIMESH_MEMBERS = <String>[MESH];

//<todo.eoin I really want to be able to call the DataError (etc.) constructors
class ColliderError extends IssueType {
  static final compoundColliderError = ColliderError._(
      'MSFT_COLLISION_PRIMITIVES_COMPOUND_COLLIDER',
      (args) => 'Collider contains multiple different geometries');
  static final noGeometryError = ColliderError._(
      'MSFT_COLLISION_PRIMITIVES_MISSING_COLLIDER',
      (args) => 'Collider contains no geometry');
  static final degenerateGeometry = ColliderError._(
      'MSFT_COLLISION_PRIMITIVES_DEGENERATE_COLLIDER',
      (args) => 'Collider geometry is degenerate',
      Severity.Information);
  static final inconsistentCapsuleGeometry = ColliderError._(
      'MSFT_COLLISION_PRIMITIVES_INVALID_CAPSULE',
      (args) => 'Capsule height is not sufficient to include radii');

  ColliderError._(String type, ErrorFunction message,
      [Severity severity = Severity.Error])
      : super(type, message, severity);
}

class MsftCollisionPrimitivesGltf extends GltfProperty {
  final SafeList<MsftCollisionPrimitivesCollider> colliders;

  MsftCollisionPrimitivesGltf._(
      this.colliders, Map<String, Object> extensions, Object extras)
      : super(extensions, extras);

  static MsftCollisionPrimitivesGltf fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, MSFT_COLLISION_PRIMITIVES_GLTF_MEMBERS, context);
    }

    SafeList<MsftCollisionPrimitivesCollider> colliders;
    final colliderMaps = getMapList(map, COLLIDERS, context);
    if (colliderMaps != null) {
      colliders = SafeList<MsftCollisionPrimitivesCollider>(
          colliderMaps.length, COLLIDERS);
      context.path.add(COLLIDERS);
      for (var i = 0; i < colliderMaps.length; i++) {
        final colliderMap = colliderMaps[i];
        context.path.add(i.toString());
        colliders[i] =
            MsftCollisionPrimitivesCollider.fromMap(colliderMap, context);
        context.path.removeLast();
      }
      context.path.removeLast();
    } else {
      colliders = SafeList<MsftCollisionPrimitivesCollider>.empty(COLLIDERS);
    }

    return MsftCollisionPrimitivesGltf._(
        colliders,
        getExtensions(map, MsftCollisionPrimitivesGltf, context),
        getExtras(map, context));
  }

  @override
  void link(Gltf gltf, Context context) {
    context.path.add(COLLIDERS);
    context.extensionCollections[colliders] =
        context.path.toList(growable: false);
    colliders.forEachWithIndices((i, collider) {
      context.path.add(i.toString());
      collider.link(gltf, context);
      context.path.removeLast();
    });
    context.path.removeLast();
  }
}

class MsftCollisionPrimitivesCollider extends GltfChildOfRootProperty {
  final List<String> collisionSystems;
  final List<String> collideWithSystems;
  final List<String> notCollideWithSystems;

  MsftCollisionPrimitivesColliderGeometry geom;

  MsftCollisionPrimitivesCollider._(
      this.collisionSystems,
      this.collideWithSystems,
      this.notCollideWithSystems,
      this.geom,
      String name,
      Map<String, Object> extensions,
      Object extras)
      : super(name, extensions, extras);

  static MsftCollisionPrimitivesCollider fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, MSFT_COLLISION_PRIMITIVES_COLLIDER_MEMBERS, context);
    }

    final collisionSystems = getStringList(map, COLLISION_SYSTEMS, context);
    final collideWithSystems =
        getStringList(map, COLLIDE_WITH_SYSTEMS, context);
    final notCollideWithSystems =
        getStringList(map, NOT_COLLIDE_WITH_SYSTEMS, context);

    const typeNames = [SPHERE, BOX, CAPSULE, CYLINDER, CONVEX, TRIMESH];
    const typeFns = [
      MsftCollisionPrimitivesColliderSphere.fromMap,
      MsftCollisionPrimitivesColliderBox.fromMap,
      MsftCollisionPrimitivesColliderCapsule.fromMap,
      MsftCollisionPrimitivesColliderCylinder.fromMap,
      MsftCollisionPrimitivesColliderConvex.fromMap,
      MsftCollisionPrimitivesColliderTrimesh.fromMap
    ];
    assert(typeNames.length == typeFns.length);

    MsftCollisionPrimitivesColliderGeometry geometry;
    for (var i = 0; i < typeNames.length; i++) {
      if (map.containsKey(typeNames[i])) {
        if (geometry != null) {
          context.addIssue(ColliderError.compoundColliderError,
              name: typeNames[i]);
        }
        geometry = getObjectFromInnerMap(map, typeNames[i], context, typeFns[i],
            req: false);
      }
    }

    final extensions =
        getExtensions(map, MsftCollisionPrimitivesCollider, context);
    if (geometry == null && (extensions == null || extensions.isEmpty)) {
      context.addIssue(ColliderError.noGeometryError);
    }

    return MsftCollisionPrimitivesCollider._(
        collisionSystems,
        collideWithSystems,
        notCollideWithSystems,
        geometry,
        getName(map, context),
        extensions,
        getExtras(map, context));
  }

  @override
  void link(Gltf gltf, Context context) {
    if (context.validate && geom != null) {
      geom.link(gltf, context);
    }
  }
}

class MsftCollisionPrimitivesColliderGeometry extends GltfProperty {
  MsftCollisionPrimitivesColliderGeometry(
      Map<String, Object> extensions, Object extras)
      : super(extensions, extras);
}

class MsftCollisionPrimitivesColliderSphere
    extends MsftCollisionPrimitivesColliderGeometry {
  final double radius;

  MsftCollisionPrimitivesColliderSphere._(
      this.radius, Map<String, Object> extensions, Object extras)
      : super(extensions, extras);

  static MsftCollisionPrimitivesColliderGeometry fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, MSFT_COLLISION_PRIMITIVES_SPHERE_MEMBERS, context);
    }

    final radius = getFloat(map, RADIUS, context, min: 0, def: 0.5);
    if (context.validate && radius == 0) {
      context.addIssue(ColliderError.degenerateGeometry, name: RADIUS);
    }
    return MsftCollisionPrimitivesColliderSphere._(
        radius,
        getExtensions(map, MsftCollisionPrimitivesColliderSphere, context),
        getExtras(map, context));
  }
}

class MsftCollisionPrimitivesColliderBox
    extends MsftCollisionPrimitivesColliderGeometry {
  final Vector3 size;

  MsftCollisionPrimitivesColliderBox._(
      this.size, Map<String, Object> extensions, Object extras)
      : super(extensions, extras);

  static MsftCollisionPrimitivesColliderGeometry fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, MSFT_COLLISION_PRIMITIVES_BOX_MEMBERS, context);
    }

    final sizeList = getFloatList(map, SIZE, context,
        min: 0, lengthsList: const [3], def: const [1.0, 1.0, 1.0]);

    if (context.validate && sizeList != null) {
      context.path.add(SIZE);
      for (var i = 0; i < 3; i++) {
        context.path.add(i.toString());
        if (sizeList[i] == 0) {
          context.addIssue(ColliderError.degenerateGeometry);
        }
        context.path.removeLast();
      }
      context.path.removeLast();
    }

    return MsftCollisionPrimitivesColliderBox._(
        _listToVec3(sizeList),
        getExtensions(map, MsftCollisionPrimitivesColliderBox, context),
        getExtras(map, context));
  }

  static Vector3 _listToVec3(List<double> l) {
    if (l != null) {
      return Vector3(l[0], l[1], l[2]);
    }
    return null;
  }
}

class MsftCollisionPrimitivesColliderCapsule
    extends MsftCollisionPrimitivesColliderGeometry {
  final double radius;
  final double height;

  MsftCollisionPrimitivesColliderCapsule._(
      this.radius, this.height, Map<String, Object> extensions, Object extras)
      : super(extensions, extras);

  static MsftCollisionPrimitivesColliderGeometry fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, MSFT_COLLISION_PRIMITIVES_CAPSULE_MEMBERS, context);
    }

    final radius = getFloat(map, RADIUS, context, min: 0, def: 0.25);
    final height = getFloat(map, HEIGHT, context, min: 0, def: 0.5);

    if (context.validate) {
      if (radius == 0) {
        context.addIssue(ColliderError.degenerateGeometry, name: RADIUS);
      }
      if (height == 0) {
        context.addIssue(ColliderError.degenerateGeometry, name: HEIGHT);
      }

      if (height - radius * 2 < 0) {
        context.addIssue(ColliderError.inconsistentCapsuleGeometry);
      } else if (height - radius * 2 < unitSumThresholdStep) {
        context.addIssue(ColliderError.degenerateGeometry);
      }
    }

    return MsftCollisionPrimitivesColliderCapsule._(
        radius,
        height,
        getExtensions(map, MsftCollisionPrimitivesColliderCapsule, context),
        getExtras(map, context));
  }
}

class MsftCollisionPrimitivesColliderCylinder
    extends MsftCollisionPrimitivesColliderGeometry {
  final double radius;
  final double height;

  MsftCollisionPrimitivesColliderCylinder._(
      this.radius, this.height, Map<String, Object> extensions, Object extras)
      : super(extensions, extras);

  static MsftCollisionPrimitivesColliderGeometry fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, MSFT_COLLISION_PRIMITIVES_CYLINDER_MEMBERS, context);
    }

    final radius = getFloat(map, RADIUS, context, min: 0, def: 0.25);
    final height = getFloat(map, HEIGHT, context, min: 0, def: 0.5);

    if (context.validate) {
      if (radius == 0) {
        context.addIssue(ColliderError.degenerateGeometry, name: RADIUS);
      }
      if (height == 0) {
        context.addIssue(ColliderError.degenerateGeometry, name: HEIGHT);
      }
    }
    return MsftCollisionPrimitivesColliderCylinder._(
        radius,
        height,
        getExtensions(map, MsftCollisionPrimitivesColliderCylinder, context),
        getExtras(map, context));
  }
}

class MsftCollisionPrimitivesColliderConvex
    extends MsftCollisionPrimitivesColliderGeometry {
  final int _meshIndex;
  Mesh _mesh;

  MsftCollisionPrimitivesColliderConvex._(
      this._meshIndex, Map<String, Object> extensions, Object extras)
      : super(extensions, extras);

  static MsftCollisionPrimitivesColliderGeometry fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, MSFT_COLLISION_PRIMITIVES_CONVEX_MEMBERS, context);
    }

    final mesh = getIndex(map, MESH, context, req: true);
    return MsftCollisionPrimitivesColliderConvex._(
        mesh,
        getExtensions(map, MsftCollisionPrimitivesColliderConvex, context),
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

class MsftCollisionPrimitivesColliderTrimesh
    extends MsftCollisionPrimitivesColliderGeometry {
  final int _meshIndex;
  Mesh _mesh;

  MsftCollisionPrimitivesColliderTrimesh._(
      this._meshIndex, Map<String, Object> extensions, Object extras)
      : super(extensions, extras);

  static MsftCollisionPrimitivesColliderGeometry fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, MSFT_COLLISION_PRIMITIVES_TRIMESH_MEMBERS, context);
    }

    final mesh = getIndex(map, MESH, context, req: true);
    return MsftCollisionPrimitivesColliderTrimesh._(
        mesh,
        getExtensions(map, MsftCollisionPrimitivesColliderCylinder, context),
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
