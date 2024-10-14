library gltf.extensions.khr_physics_rigid_bodies;

import 'package:gltf/src/base/gltf_property.dart';
import 'package:gltf/src/ext/KHR_implicit_shapes/KHR_implicit_shapes.dart';
import 'package:gltf/src/ext/extensions.dart';
import 'package:vector_math/vector_math.dart';

const String KHR_PHYSICS_RIGID_BODIES = 'KHR_physics_rigid_bodies';

const Extension khrPhysicsRigidBodiesExtension =
    Extension(KHR_PHYSICS_RIGID_BODIES, <Type, ExtensionDescriptor>{
  Gltf: ExtensionDescriptor(KhrPhysicsRigidBodiesGltf.fromMap),
  Node: ExtensionDescriptor(KhrPhysicsRigidBodiesNode.fromMap),
});

class RigidBodiesError extends IssueType {
  static final incorrectJointRange = RigidBodiesError._(
      'KHR_RIGID_BODIES_JOINT_INVALID_RANGE',
      (args) => 'Joint limit min (${args[0]}) should be less or equal to '
          'max (${args[1]})');

  static final incorrectLimitLength = RigidBodiesError._(
      'KHR_RIGID_BODIES_JOINT_LIMITS_INCORRECT_SIZE',
      (args) => 'Length of limited axes should be in range [1,3)');

  static final limitIndexOutOfBounds = RigidBodiesError._(
      'KHR_RIGID_BODIES_JOINT_LIMITS_INVALID_AXIS',
      (args) => 'Indices of limited axes should be in range [0,3)');

  static final collideAndNotCollideSpecified = RigidBodiesError._(
      'KHR_RIGID_BODIES_COLLISION_FILTER_INVALID_FILTER_SET',
      (args) =>
          'Only one of collide-with and not-collide-with should be provided');

  static final invalidChildTrigger = RigidBodiesError._(
      'KHR_RIGID_BODIES_TRIGGER_NODES_INVALID',
      (args) => 'Trigger nodes should be simple triggers');

  RigidBodiesError._(String type, ErrorFunction message,
      [Severity severity = Severity.Error])
      : super(type, message, severity);
}

const String PHYSICS_MATERIALS = 'physicsMaterials';
const String PHYSICS_JOINTS = 'physicsJoints';
const String COLLISION_FILTERS = 'collisionFilters';
const String STATIC_FRICTION = 'staticFriction';
const String DYNAMIC_FRICTION = 'dynamicFriction';
const String RESTITUTION = 'restitution';
const String FRICTION_COMBINE = 'frictionCombine';
const String RESTITUTION_COMBINE = 'restitutionCombine';
const String AVERAGE = 'average';
const String MINIMUM = 'minimum';
const String MAXIMUM = 'maximum';
const String MULTIPLY = 'multiply';
const String COLLISION_SYSTEMS = 'collisionSystems';
const String COLLIDE_WITH_SYSTEMS = 'collideWithSystems';
const String NOT_COLLIDE_WITH_SYSTEMS = 'notCollideWithSystems';
const String COLLIDER = 'collider';
const String SHAPE = 'shape';
const String CONVEX_HULL = 'convexHull';
const String GEOMETRY = 'geometry';
const String PHYSICS_MATERIAL = 'physicsMaterial';
const String COLLISION_FILTER = 'collisionFilter';
const String TRIGGER = 'trigger';
const String JOINT = 'joint';
const String MOTION = 'motion';
const String IS_KINEMATIC = 'isKinematic';
const String MASS = 'mass';
const String CENTER_OF_MASS = 'centerOfMass';
const String INERTIA_ORIENTATION = 'inertiaOrientation';
const String INERTIA_DIAGONAL = 'inertiaDiagonal';
const String LINEAR_VELOCITY = 'linearVelocity';
const String ANGULAR_VELOCITY = 'angularVelocity';
const String GRAVITY_FACTOR = 'gravityFactor';
const String CONNECTED_NODE = 'connectedNode';
const String ENABLE_COLLISION = 'enableCollision';
const String LINEAR_AXES = 'linearAxes';
const String ANGULAR_AXES = 'angularAxes';
const String LIMITS = 'limits';
const String DRIVES = 'drives';
const String MODE = 'mode';
const String AXIS = 'axis';
const String POSITION_TARGET = 'positionTarget';
const String VELOCITY_TARGET = 'velocityTarget';
const String STIFFNESS = 'stiffness';
const String DAMPING = 'damping';
const String MAX_FORCE = 'maxForce';
const String LINEAR = 'linear';
const String ANGULAR = 'angular';
const String FORCE = 'force';
const String ACCELERATION = 'acceleration';

const List<String> KHR_RIGID_BODIES_GLTF_MEMBERS = <String>[
  PHYSICS_MATERIALS,
  PHYSICS_JOINTS,
  COLLISION_FILTERS
];
const List<String> KHR_RIGID_BODIES_PHYSICS_MATERIAL_MEMBERS = <String>[
  STATIC_FRICTION,
  DYNAMIC_FRICTION,
  RESTITUTION,
  FRICTION_COMBINE,
  RESTITUTION_COMBINE
];
const List<String> KHR_MATERIAL_COMBINE_MODES = <String>[
  AVERAGE,
  MINIMUM,
  MAXIMUM,
  MULTIPLY
];
const List<String> KHR_RIGIBODIES_COLLISION_FILTER_MEMBERS = <String>[
  COLLISION_SYSTEMS,
  COLLIDE_WITH_SYSTEMS,
  NOT_COLLIDE_WITH_SYSTEMS
];
const List<String> KHR_RIGID_BODIES_JOINT_LIMIT_SET_MEMBERS = <String>[
  LIMITS,
  DRIVES
];
const List<String> KHR_RIGID_BODIES_JOINT_LIMIT_MEMBERS = <String>[
  MIN,
  MAX,
  STIFFNESS,
  DAMPING,
  LINEAR_AXES,
  ANGULAR_AXES
];
const List<String> KHR_RIGID_BODIES_NODE_MEMBERS = <String>[
  MOTION,
  COLLIDER,
  TRIGGER,
  JOINT
];
const List<String> KHR_RIGID_BODIES_MOTION_MEMBERS = <String>[
  IS_KINEMATIC,
  MASS,
  CENTER_OF_MASS,
  INERTIA_ORIENTATION,
  INERTIA_DIAGONAL,
  LINEAR_VELOCITY,
  ANGULAR_VELOCITY,
  GRAVITY_FACTOR
];
const List<String> KHR_RIGID_BODIES_SHAPE_EXTENSION_MEMBERS = <String>[
  CONVEX_HULL
];
const List<String> KHR_RIGID_BODIES_GEOMETRY_MEMBERS = <String>[
  SHAPE,
  NODE,
  CONVEX_HULL
];
const List<String> KHR_RIGID_BODIES_COLLIDER_MEMBERS = <String>[
  GEOMETRY,
  PHYSICS_MATERIAL,
  COLLISION_FILTER
];
const List<String> KHR_RIGID_BODIES_TRIGGER_MEMBERS = <String>[
  GEOMETRY,
  COLLISION_FILTER,
  NODES
];
const List<String> KHR_RIGID_BODIES_JOINT_MEMBERS = <String>[
  CONNECTED_NODE,
  JOINT,
  ENABLE_COLLISION
];
const List<String> KHR_RIGID_BODIES_JOINT_DRIVE_MEMBERS = <String>[
  TYPE,
  MODE,
  AXIS,
  MAX_FORCE,
  VELOCITY_TARGET,
  DAMPING,
  POSITION_TARGET,
  STIFFNESS
];
const List<String> KHR_RIGID_BODIES_JOINT_DRIVE_TYPES = <String>[
  LINEAR,
  ANGULAR
];
const List<String> KHR_RIGID_BODIES_JOINT_DRIVE_MODES = <String>[
  FORCE,
  ACCELERATION
];

class KhrPhysicsRigidBodiesGltf extends GltfProperty {
  final SafeList<KhrPhysicsRigidBodiesPhysicsMaterial> physicsMaterials;
  final SafeList<KhrPhysicsRigidBodiesCollisionFilter> collisionFilters;
  final SafeList<KhrPhysicsRigidBodiesPhysicsJointDefinition> jointDefinitions;

  KhrPhysicsRigidBodiesGltf._(this.physicsMaterials, this.collisionFilters,
      this.jointDefinitions, Map<String, Object> extensions, Object extras)
      : super(extensions, extras);

  static KhrPhysicsRigidBodiesGltf fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, KHR_RIGID_BODIES_GLTF_MEMBERS, context);
    }

    var materials =
        SafeList<KhrPhysicsRigidBodiesPhysicsMaterial>.empty(PHYSICS_MATERIALS);
    if (map.containsKey(PHYSICS_MATERIALS)) {
      final materialMaps = getMapList(map, PHYSICS_MATERIALS, context);
      if (materialMaps != null) {
        materials = SafeList<KhrPhysicsRigidBodiesPhysicsMaterial>(
            materialMaps.length, PHYSICS_MATERIALS);
        context.path.add(PHYSICS_MATERIALS);
        for (var i = 0; i < materialMaps.length; i++) {
          context.path.add(i.toString());
          materials[i] = KhrPhysicsRigidBodiesPhysicsMaterial.fromMap(
              materialMaps[i], context);
          context.path.removeLast();
        }
        context.path.removeLast();
      }
    }

    var jointDefinitions =
        SafeList<KhrPhysicsRigidBodiesPhysicsJointDefinition>.empty(
            PHYSICS_JOINTS);
    if (map.containsKey(PHYSICS_JOINTS)) {
      final jointDefMaps = getMapList(map, PHYSICS_JOINTS, context);
      if (jointDefMaps != null) {
        jointDefinitions =
            SafeList<KhrPhysicsRigidBodiesPhysicsJointDefinition>(
                jointDefMaps.length, PHYSICS_JOINTS);
        context.path.add(PHYSICS_JOINTS);
        for (var i = 0; i < jointDefMaps.length; i++) {
          context.path.add(i.toString());
          jointDefinitions[i] =
              KhrPhysicsRigidBodiesPhysicsJointDefinition.fromMap(
                  jointDefMaps[i], context);
          context.path.removeLast();
        }
        context.path.removeLast();
      }
    }

    var filters =
        SafeList<KhrPhysicsRigidBodiesCollisionFilter>.empty(COLLISION_FILTERS);
    if (map.containsKey(COLLISION_FILTERS)) {
      final filterListMaps = getMapList(map, COLLISION_FILTERS, context);
      if (filterListMaps != null) {
        filters = SafeList<KhrPhysicsRigidBodiesCollisionFilter>(
            filterListMaps.length, COLLISION_FILTERS);
        context.path.add(COLLISION_FILTERS);
        for (var i = 0; i < filterListMaps.length; i++) {
          context.path.add(i.toString());
          filters[i] = KhrPhysicsRigidBodiesCollisionFilter.fromMap(
              filterListMaps[i], context);
          context.path.removeLast();
        }
        context.path.removeLast();
      }
    }

    return KhrPhysicsRigidBodiesGltf._(
        materials,
        filters,
        jointDefinitions,
        getExtensions(map, KhrPhysicsRigidBodiesGltf, context),
        getExtras(map, context));
  }

  @override
  void link(Gltf gltf, Context context) {
    void linkListItems<T extends GltfProperty>(String name, SafeList<T> list) {
      context.path.add(name);
      context.extensionCollections[list] = context.path.toList(growable: false);
      list.forEachWithIndices((i, item) {
        context.path.add(i.toString());
        item.link(gltf, context);
        context.path.removeLast();
      });
      context.path.removeLast();
    }

    linkListItems(PHYSICS_MATERIALS, physicsMaterials);
    linkListItems(COLLISION_FILTERS, collisionFilters);
    linkListItems(PHYSICS_JOINTS, jointDefinitions);
  }
}

class KhrPhysicsRigidBodiesNode extends GltfProperty {
  KhrPhysicsRigidBodiesMotion motion;
  KhrPhysicsRigidBodiesCollider collider;
  KhrPhysicsRigidBodiesTrigger trigger;
  KhrPhysicsRigidBodiesJoint joint;

  KhrPhysicsRigidBodiesNode._(this.motion, this.collider, this.trigger,
      this.joint, Map<String, Object> extensions, Object extras)
      : super(extensions, extras);

  static KhrPhysicsRigidBodiesNode fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, KHR_RIGID_BODIES_NODE_MEMBERS, context);
    }

    final motion = getObjectFromInnerMap(
        map, MOTION, context, KhrPhysicsRigidBodiesMotion.fromMap,
        req: false);
    final collider = getObjectFromInnerMap(
        map, COLLIDER, context, KhrPhysicsRigidBodiesCollider.fromMap,
        req: false);
    final trigger = getObjectFromInnerMap(
        map, TRIGGER, context, KhrPhysicsRigidBodiesTrigger.fromMap,
        req: false);
    final joint = getObjectFromInnerMap(
        map, JOINT, context, KhrPhysicsRigidBodiesJoint.fromMap,
        req: false);

    return KhrPhysicsRigidBodiesNode._(
        motion,
        collider,
        trigger,
        joint,
        getExtensions(map, KhrPhysicsRigidBodiesNode, context),
        getExtras(map, context));
  }

  @override
  void link(Gltf gltf, Context context) {
    void linkObject<T extends GltfProperty>(String name, T obj) {
      if (obj != null) {
        context.path.add(name);
        obj.link(gltf, context);
        context.path.removeLast();
      }
    }

    linkObject(MOTION, motion);
    linkObject(COLLIDER, collider);
    linkObject(TRIGGER, trigger);
    linkObject(JOINT, joint);
  }
}

class KhrPhysicsRigidBodiesGeometry extends GltfProperty {
  final int _shapeIndex;
  KhrImplicitShapesShape _shape;
  final int _nodeIndex;
  Node _node;
  final bool convexHull;

  KhrPhysicsRigidBodiesGeometry._(this._shapeIndex, this._nodeIndex,
      this.convexHull, Map<String, Object> extensions, Object extras)
      : super(extensions, extras);

  static KhrPhysicsRigidBodiesGeometry fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, KHR_RIGID_BODIES_GEOMETRY_MEMBERS, context);
    }

    final shape = getIndex(map, SHAPE, context, req: false);
    final node = getIndex(map, NODE, context, req: false);
    final convexHull = getBool(map, CONVEX_HULL, context);
    return KhrPhysicsRigidBodiesGeometry._(
        shape,
        node,
        convexHull,
        getExtensions(map, KhrPhysicsRigidBodiesGeometry, context),
        getExtras(map, context));
  }

  @override
  void link(Gltf gltf, Context context) {
    _shape = getShapeByIndex(gltf, context, _shapeIndex);
    if (_shape != null) {
      context.path.add(SHAPE);
      _shape.link(gltf, context);
      context.path.removeLast();
    }

    if (_nodeIndex != -1) {
      final nodes = List<Node>.filled(1, null);
      resolveNodeList(
          [_nodeIndex], nodes, gltf.nodes, NODE, context, (_n, _ni, _i) {});
      _node = nodes[0];
    }
  }

  KhrImplicitShapesShape get shape => _shape;
  Node get node => _node;
}

class KhrPhysicsRigidBodiesCollider extends GltfProperty {
  final KhrPhysicsRigidBodiesGeometry geometry;
  final int _physicsMaterialIndex;
  KhrPhysicsRigidBodiesPhysicsMaterial _material;
  final int _collisionFilterIndex;
  KhrPhysicsRigidBodiesCollisionFilter _collisionFilter;

  KhrPhysicsRigidBodiesCollider._(this.geometry, this._physicsMaterialIndex,
      this._collisionFilterIndex, Map<String, Object> extensions, Object extras)
      : super(extensions, extras);

  static KhrPhysicsRigidBodiesCollider fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, KHR_RIGID_BODIES_COLLIDER_MEMBERS, context);
    }

    final geometry = getObjectFromInnerMap(
        map, GEOMETRY, context, KhrPhysicsRigidBodiesGeometry.fromMap,
        req: true);
    final material = getIndex(map, PHYSICS_MATERIAL, context, req: false);
    final filter = getIndex(map, COLLISION_FILTER, context, req: false);
    return KhrPhysicsRigidBodiesCollider._(
        geometry,
        material,
        filter,
        getExtensions(map, KhrPhysicsRigidBodiesCollider, context),
        getExtras(map, context));
  }

  @override
  void link(Gltf gltf, Context context) {
    context.path.add(GEOMETRY);
    geometry.link(gltf, context);
    context.path.removeLast();
    _collisionFilter =
        getCollisionFilterByIndex(gltf, context, _collisionFilterIndex);
    if (_collisionFilter != null) {
      context.path.add(COLLISION_FILTER);
      _collisionFilter.link(gltf, context);
      context.path.removeLast();
    }
    _material = getPhysicsMaterialByIndex(gltf, context, _physicsMaterialIndex);
    if (_material != null) {
      context.path.add(PHYSICS_MATERIAL);
      _material.link(gltf, context);
      context.path.removeLast();
    }
  }

  KhrPhysicsRigidBodiesPhysicsMaterial get material => _material;
  KhrPhysicsRigidBodiesCollisionFilter get collisionFilter => _collisionFilter;
}

class KhrPhysicsRigidBodiesTrigger extends GltfProperty {
  final KhrPhysicsRigidBodiesGeometry geometry;
  final int _collisionFilterIndex;
  final List<int> _nodes;
  KhrPhysicsRigidBodiesCollisionFilter _collisionFilter;

  KhrPhysicsRigidBodiesTrigger._(this.geometry, this._collisionFilterIndex,
      this._nodes, Map<String, Object> extensions, Object extras)
      : super(extensions, extras);

  static KhrPhysicsRigidBodiesTrigger fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, KHR_RIGID_BODIES_TRIGGER_MEMBERS, context);
    }

    final nodes = getIndicesList(map, NODES, context, req: false);
    final geometry = getObjectFromInnerMap(
        map, GEOMETRY, context, KhrPhysicsRigidBodiesGeometry.fromMap,
        req: false);
    final filter = getIndex(map, COLLISION_FILTER, context, req: false);
    if ((nodes != null) == (geometry != null)) {
      context.addIssue(SchemaError.oneOfMismatch, args: [NODES, GEOMETRY]);
    }
    if ((nodes != null) && (filter != -1)) {
      context
          .addIssue(SchemaError.oneOfMismatch, args: [NODES, COLLISION_FILTER]);
    }

    return KhrPhysicsRigidBodiesTrigger._(
        geometry,
        filter,
        nodes,
        getExtensions(map, KhrPhysicsRigidBodiesTrigger, context),
        getExtras(map, context));
  }

  @override
  void link(Gltf gltf, Context context) {
    if (geometry != null) {
      context.path.add(GEOMETRY);
      geometry.link(gltf, context);
      context.path.removeLast();
    }
    _collisionFilter =
        getCollisionFilterByIndex(gltf, context, _collisionFilterIndex);
    if (_collisionFilter != null) {
      context.path.add(COLLISION_FILTER);
      _collisionFilter.link(gltf, context);
      context.path.removeLast();
    }
    if (_nodes != null) {
      context.path.add(NODES);
      for (var i = 0; i < _nodes.length; i++) {
        final nodeI = gltf.nodes[_nodes[i]];

        final rbExt = nodeI.extensions[khrPhysicsRigidBodiesExtension.name];
        if (rbExt is KhrPhysicsRigidBodiesNode &&
            rbExt.trigger != null &&
            rbExt.trigger.isSimple) {
          nodeI.markAsUsed();
        } else {
          context.path.add(i.toString());
          context.addIssue(RigidBodiesError.invalidChildTrigger);
          context.path.removeLast();
        }
      }
      context.path.removeLast();
    }
  }

  bool get isComposite => _nodes != null && geometry == null;
  bool get isSimple => _nodes == null && geometry != null;
  KhrPhysicsRigidBodiesCollisionFilter get collisionFilter => _collisionFilter;
}

KhrImplicitShapesShape getShapeByIndex(
    Gltf gltf, Context context, int shapeIndex) {
  final implicitShapesExtension =
      gltf.extensions[khrImplicitShapesExtension.name];
  if (implicitShapesExtension is KhrImplicitShapesGltf) {
    final shape = implicitShapesExtension.shapes[shapeIndex];
    if (shapeIndex != -1) {
      if (context.validate && shape == null) {
        context.addIssue(LinkError.unresolvedReference,
            name: SHAPE, args: [shapeIndex]);
      } else if (shape != null) {
        shape.markAsUsed();
      }
    }
    return shape;
  } else if (context.validate && shapeIndex >= 0) {
    context.addIssue(SchemaError.unsatisfiedDependency,
        args: ['/$EXTENSIONS/${khrImplicitShapesExtension.name}']);
  }
  return null;
}

KhrPhysicsRigidBodiesCollisionFilter getCollisionFilterByIndex(
    Gltf gltf, Context context, int filterIndex) {
  final rigidBodiesExtension =
      gltf.extensions[khrPhysicsRigidBodiesExtension.name];
  if (rigidBodiesExtension is KhrPhysicsRigidBodiesGltf) {
    final filter = rigidBodiesExtension.collisionFilters[filterIndex];
    if (filterIndex != -1) {
      if (context.validate && filter == null) {
        context.addIssue(LinkError.unresolvedReference,
            name: COLLISION_FILTER, args: [filterIndex]);
      } else if (filter != null) {
        filter.markAsUsed();
      }
    }
    return filter;
  } else if (context.validate && filterIndex >= 0) {
    context.addIssue(SchemaError.unsatisfiedDependency,
        args: ['/$EXTENSIONS/${khrPhysicsRigidBodiesExtension.name}']);
  }
  return null;
}

KhrPhysicsRigidBodiesPhysicsMaterial getPhysicsMaterialByIndex(
    Gltf gltf, Context context, int materialIndex) {
  final rigidBodiesExtension =
      gltf.extensions[khrPhysicsRigidBodiesExtension.name];
  if (rigidBodiesExtension is KhrPhysicsRigidBodiesGltf) {
    final material = rigidBodiesExtension.physicsMaterials[materialIndex];
    if (materialIndex != -1) {
      if (context.validate && material == null) {
        context.addIssue(LinkError.unresolvedReference,
            name: PHYSICS_MATERIAL, args: [materialIndex]);
      } else if (material != null) {
        material.markAsUsed();
      }
    }
    return material;
  } else if (context.validate && materialIndex >= 0) {
    context.addIssue(SchemaError.unsatisfiedDependency,
        args: ['/$EXTENSIONS/${khrPhysicsRigidBodiesExtension.name}']);
  }
  return null;
}

class KhrPhysicsRigidBodiesMotion extends GltfProperty {
  bool isKinematic;
  double mass;
  Vector3 centerOfMass;
  Vector3 inertiaDiagonal;
  Quaternion inertiaOrientation;
  Vector3 linearVelocity;
  Vector3 angularVelocity;
  double gravityFactor;

  KhrPhysicsRigidBodiesMotion._(
      this.isKinematic,
      this.mass,
      this.centerOfMass,
      this.inertiaDiagonal,
      this.inertiaOrientation,
      this.linearVelocity,
      this.angularVelocity,
      this.gravityFactor,
      Map<String, Object> extensions,
      Object extras)
      : super(extensions, extras);

  static KhrPhysicsRigidBodiesMotion fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, KHR_RIGID_BODIES_MOTION_MEMBERS, context);
    }

    final isKinematic = getBool(map, IS_KINEMATIC, context);
    final mass = getFloat(map, MASS, context, req: false, def: 1, min: 0);

    final comList = getFloatList(map, CENTER_OF_MASS, context,
        lengthsList: const [3], def: const [0.0, 0.0, 0.0]);
    final com = _listToVec3(comList);

    Quaternion inertiaOrientation;
    final inertiaOrientationList = getFloatList(
        map, INERTIA_ORIENTATION, context,
        lengthsList: const [4], def: const [0.0, 0.0, 0.0, 1.0]);
    if (inertiaOrientationList != null) {
      inertiaOrientation = Quaternion(
          inertiaOrientationList[0],
          inertiaOrientationList[1],
          inertiaOrientationList[2],
          inertiaOrientationList[3]);
      if (context.validate &&
          (1.0 - inertiaOrientation.length).abs() > unitLengthThresholdVec4) {
        context.addIssue(SemanticError.rotationNonUnit,
            name: INERTIA_ORIENTATION);
      }
    }

    final inertiaDiagList = getFloatList(map, INERTIA_DIAGONAL, context,
        min: 0, lengthsList: const [3], def: [1.0, 1.0, 1.0]);
    final inertiaDiagonal = _listToVec3(inertiaDiagList);

    final linearVelocityList = getFloatList(map, LINEAR_VELOCITY, context,
        lengthsList: const [3], def: [0.0, 0.0, 0.0]);
    final linearVelocity = _listToVec3(linearVelocityList);

    final angularVelocityList = getFloatList(map, ANGULAR_VELOCITY, context,
        lengthsList: const [3], def: [0.0, 0.0, 0.0]);
    final angularVelocity = _listToVec3(angularVelocityList);

    final gravityFactor =
        getFloat(map, GRAVITY_FACTOR, context, req: false, def: 1);

    return KhrPhysicsRigidBodiesMotion._(
        isKinematic,
        mass,
        com,
        inertiaDiagonal,
        inertiaOrientation,
        linearVelocity,
        angularVelocity,
        gravityFactor,
        getExtensions(map, KhrPhysicsRigidBodiesNode, context),
        getExtras(map, context));
  }

  static Vector3 _listToVec3(List<double> l) {
    if (l != null) {
      return Vector3(l[0], l[1], l[2]);
    }
    return null;
  }
}

class KhrPhysicsRigidBodiesJoint extends GltfProperty {
  final int _connectedNodeIndex;
  Node _connectedNode;
  final int _jointDefinitionIndex;
  KhrPhysicsRigidBodiesPhysicsJointDefinition _jointDef;
  bool enableCollision;

  KhrPhysicsRigidBodiesJoint._(
      this._connectedNodeIndex,
      this._jointDefinitionIndex,
      this.enableCollision,
      Map<String, Object> extensions,
      Object extras)
      : super(extensions, extras);

  static KhrPhysicsRigidBodiesJoint fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, KHR_RIGID_BODIES_JOINT_MEMBERS, context);
    }

    final connectedNode = getIndex(map, CONNECTED_NODE, context, req: false);
    final jointIndex = getIndex(map, JOINT, context, req: true);
    final enableCollision = getBool(map, ENABLE_COLLISION, context);

    return KhrPhysicsRigidBodiesJoint._(
        connectedNode,
        jointIndex,
        enableCollision,
        getExtensions(map, KhrPhysicsRigidBodiesNode, context),
        getExtras(map, context));
  }

  @override
  void link(Gltf gltf, Context context) {
    _connectedNode = gltf.nodes[_connectedNodeIndex];
    if (_connectedNodeIndex != -1) {
      if (context.validate && _connectedNode == null) {
        context.addIssue(LinkError.unresolvedReference,
            name: CONNECTED_NODE, args: [_connectedNodeIndex]);
      } else if (_connectedNode != null) {
        _connectedNode.markAsUsed();
      }
    }

    final rigidBodiesExtension =
        gltf.extensions[khrPhysicsRigidBodiesExtension.name];
    if (rigidBodiesExtension is KhrPhysicsRigidBodiesGltf) {
      _jointDef = rigidBodiesExtension.jointDefinitions[_jointDefinitionIndex];
      if (_jointDefinitionIndex != -1) {
        if (context.validate && _jointDef == null) {
          context.addIssue(LinkError.unresolvedReference,
              name: JOINT, args: [_jointDefinitionIndex]);
        } else if (_jointDef != null) {
          _jointDef.markAsUsed();
        }
      }
    } else if (context.validate && _jointDefinitionIndex != -1) {
      context.addIssue(SchemaError.unsatisfiedDependency,
          args: ['/$EXTENSIONS/${khrPhysicsRigidBodiesExtension.name}']);
    }
  }

  Node get connectedNode => _connectedNode;
  KhrPhysicsRigidBodiesPhysicsJointDefinition get jointDefinition => _jointDef;
}

class KhrPhysicsRigidBodiesPhysicsMaterial extends GltfChildOfRootProperty {
  final double staticFriction;
  final double dynamicFriction;
  final double restitution;
  final String frictionCombine;
  final String restitutionCombine;

  KhrPhysicsRigidBodiesPhysicsMaterial._(
      this.staticFriction,
      this.dynamicFriction,
      this.restitution,
      this.frictionCombine,
      this.restitutionCombine,
      String name,
      Map<String, Object> extensions,
      Object extras)
      : super(name, extensions, extras);

  static KhrPhysicsRigidBodiesPhysicsMaterial fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, KHR_RIGID_BODIES_PHYSICS_MATERIAL_MEMBERS, context);
    }

    final staticFriction =
        getFloat(map, STATIC_FRICTION, context, min: 0, def: 0.6);
    final dynamicFriction =
        getFloat(map, DYNAMIC_FRICTION, context, min: 0, def: 0.6);
    final restitution = getFloat(map, RESTITUTION, context, min: 0, def: 0.6);
    final frictionCombine = getString(map, FRICTION_COMBINE, context,
        list: KHR_MATERIAL_COMBINE_MODES);
    final restitutionCombine = getString(map, RESTITUTION_COMBINE, context,
        list: KHR_MATERIAL_COMBINE_MODES);

    return KhrPhysicsRigidBodiesPhysicsMaterial._(
        staticFriction,
        dynamicFriction,
        restitution,
        frictionCombine,
        restitutionCombine,
        getName(map, context),
        getExtensions(map, KhrPhysicsRigidBodiesPhysicsMaterial, context),
        getExtras(map, context));
  }
}

class KhrPhysicsRigidBodiesPhysicsJointLimit extends GltfProperty {
  double min;
  double max;
  double stiffness;
  double damping;
  List<int> linearAxes;
  List<int> angularAxes;
  KhrPhysicsRigidBodiesPhysicsJointLimit._(
      this.min,
      this.max,
      this.stiffness,
      this.damping,
      this.linearAxes,
      this.angularAxes,
      Map<String, Object> extensions,
      Object extras)
      : super(extensions, extras);

  static KhrPhysicsRigidBodiesPhysicsJointLimit fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, KHR_RIGID_BODIES_JOINT_LIMIT_MEMBERS, context);
    }

    final minLimit = getFloat(map, MIN, context);
    final maxLimit = getFloat(map, MAX, context);
    final stiffness = getFloat(map, STIFFNESS, context, min: 0);
    final damping = getFloat(map, DAMPING, context, min: 0);
    final linLimits = getIndicesList(map, LINEAR_AXES, context);
    final angLimits = getIndicesList(map, ANGULAR_AXES, context);

    if (context.validate) {
      if (!((linLimits == null) ^ (angLimits == null))) {
        context.addIssue(SchemaError.oneOfMismatch,
            args: [LINEAR_AXES, ANGULAR_AXES]);
      }

      if (minLimit > maxLimit) {
        context.addIssue(RigidBodiesError.incorrectJointRange,
            args: [minLimit, maxLimit]);
      }

      _validateLimitIndices(linLimits, LINEAR_AXES, context);
      _validateLimitIndices(angLimits, ANGULAR_AXES, context);
    }

    return KhrPhysicsRigidBodiesPhysicsJointLimit._(
        minLimit,
        maxLimit,
        stiffness,
        damping,
        linLimits,
        angLimits,
        getExtensions(map, KhrPhysicsRigidBodiesPhysicsJointLimit, context),
        getExtras(map, context));
  }

  static void _validateLimitIndices(
      List<int> limits, String name, Context context) {
    if (limits != null) {
      for (var i = 0; i < limits.length; i++) {
        if (limits[i] > 2 || limits[i] < 0) {
          context.addIssue(RigidBodiesError.limitIndexOutOfBounds, index: i);
        }
      }
      if (limits.isEmpty || limits.length > 3) {
        context.addIssue(RigidBodiesError.incorrectLimitLength, name: name);
      }
    }
  }
}

class KhrPhysicsRigidBodiesPhysicsJointDrive extends GltfProperty {
  String type;
  String mode;
  int axis;
  double maxForce;
  double velocityTarget;
  double damping;
  double positionTarget;
  double stiffness;

  KhrPhysicsRigidBodiesPhysicsJointDrive._(
      this.type,
      this.mode,
      this.axis,
      this.maxForce,
      this.velocityTarget,
      this.damping,
      this.positionTarget,
      this.stiffness,
      Map<String, Object> extensions,
      Object extras)
      : super(extensions, extras);

  static KhrPhysicsRigidBodiesPhysicsJointDrive fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, KHR_RIGID_BODIES_JOINT_DRIVE_MEMBERS, context);
    }

    final type = getString(map, TYPE, context,
        list: KHR_RIGID_BODIES_JOINT_DRIVE_TYPES, req: true);
    final mode = getString(map, MODE, context,
        list: KHR_RIGID_BODIES_JOINT_DRIVE_MODES, req: true);
    final axis = getUint(map, AXIS, context, req: true, min: 0, max: 2);
    final maxForce = getFloat(map, MAX_FORCE, context, min: 0);
    final velocityTarget = getFloat(map, VELOCITY_TARGET, context);
    final damping = getFloat(map, DAMPING, context, def: 0, min: 0);
    final positionTarget = getFloat(map, POSITION_TARGET, context);
    final stiffness = getFloat(map, STIFFNESS, context, def: 0, min: 0);

    return KhrPhysicsRigidBodiesPhysicsJointDrive._(
        type,
        mode,
        axis,
        maxForce,
        velocityTarget,
        damping,
        positionTarget,
        stiffness,
        getExtensions(map, KhrPhysicsRigidBodiesPhysicsJointDrive, context),
        getExtras(map, context));
  }
}

class KhrPhysicsRigidBodiesPhysicsJointDefinition
    extends GltfChildOfRootProperty {
  List<KhrPhysicsRigidBodiesPhysicsJointLimit> limits;
  List<KhrPhysicsRigidBodiesPhysicsJointDrive> drives;

  KhrPhysicsRigidBodiesPhysicsJointDefinition._(this.limits, this.drives,
      String name, Map<String, Object> extensions, Object extras)
      : super(name, extensions, extras);

  static KhrPhysicsRigidBodiesPhysicsJointDefinition fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, KHR_RIGID_BODIES_JOINT_LIMIT_SET_MEMBERS, context);
    }

    var limits = SafeList<KhrPhysicsRigidBodiesPhysicsJointLimit>.empty(LIMITS);
    if (map.containsKey(LIMITS)) {
      final jointLimitMaps = getMapList(map, LIMITS, context);
      if (jointLimitMaps != null) {
        limits = SafeList<KhrPhysicsRigidBodiesPhysicsJointLimit>(
            jointLimitMaps.length, LIMITS);
        context.path.add(LIMITS);
        for (var i = 0; i < jointLimitMaps.length; i++) {
          context.path.add(i.toString());
          limits[i] = KhrPhysicsRigidBodiesPhysicsJointLimit.fromMap(
              jointLimitMaps[i], context);
          context.path.removeLast();
        }
        context.path.removeLast();
      }
    }

    var drives = SafeList<KhrPhysicsRigidBodiesPhysicsJointDrive>.empty(DRIVES);
    if (map.containsKey(DRIVES)) {
      final jointDriveMaps = getMapList(map, DRIVES, context);
      if (jointDriveMaps != null) {
        drives = SafeList<KhrPhysicsRigidBodiesPhysicsJointDrive>(
            jointDriveMaps.length, DRIVES);
        context.path.add(DRIVES);
        for (var i = 0; i < jointDriveMaps.length; i++) {
          context.path.add(i.toString());
          drives[i] = KhrPhysicsRigidBodiesPhysicsJointDrive.fromMap(
              jointDriveMaps[i], context);
          context.path.removeLast();
        }
        context.path.removeLast();
      }
    }

    return KhrPhysicsRigidBodiesPhysicsJointDefinition._(
        limits,
        drives,
        getName(map, context),
        getExtensions(
            map, KhrPhysicsRigidBodiesPhysicsJointDefinition, context),
        getExtras(map, context));
  }
}

class KhrPhysicsRigidBodiesCollisionFilter extends GltfChildOfRootProperty {
  SafeList<String> collisionSystems;
  SafeList<String> collideWithSystems;
  SafeList<String> notCollideWithSystems;

  KhrPhysicsRigidBodiesCollisionFilter._(
      this.collisionSystems,
      this.collideWithSystems,
      this.notCollideWithSystems,
      String name,
      Map<String, Object> extensions,
      Object extras)
      : super(name, extensions, extras);

  static KhrPhysicsRigidBodiesCollisionFilter fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, KHR_RIGIBODIES_COLLISION_FILTER_MEMBERS, context);

      if (map.containsKey(COLLIDE_WITH_SYSTEMS) &&
          map.containsKey(NOT_COLLIDE_WITH_SYSTEMS)) {
        context.addIssue(RigidBodiesError.collideAndNotCollideSpecified);
      }
    }

    SafeList<String> extractSystemList(
        Map<String, Object> map, String keyName) {
      var listOut = SafeList<String>.empty(keyName);
      if (map.containsKey(keyName)) {
        final systemList =
            getStringList(map, keyName, context, allowEmpty: true);
        listOut = SafeList<String>(systemList.length, keyName);
        for (var i = 0; i < systemList.length; i++) {
          listOut[i] = systemList[i];
        }
      }
      return listOut;
    }

    final collisionSystems = extractSystemList(map, COLLISION_SYSTEMS);
    final collideWithSystems = extractSystemList(map, COLLIDE_WITH_SYSTEMS);
    final notCollideWithSystems =
        extractSystemList(map, NOT_COLLIDE_WITH_SYSTEMS);

    return KhrPhysicsRigidBodiesCollisionFilter._(
        collisionSystems,
        collideWithSystems,
        notCollideWithSystems,
        getName(map, context),
        getExtensions(map, KhrPhysicsRigidBodiesCollisionFilter, context),
        getExtras(map, context));
  }
}
