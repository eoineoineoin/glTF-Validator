library gltf.extensions.khr_rigid_bodies;

import 'package:gltf/src/base/gltf_property.dart';
import 'package:gltf/src/ext/KHR_collision_shapes/KHR_collision_shapes.dart';
import 'package:gltf/src/ext/extensions.dart';
import 'package:vector_math/vector_math.dart';

const String KHR_RIGID_BODIES = 'KHR_rigid_bodies';

const Extension khrRigidBodiesExtension =
    Extension(KHR_RIGID_BODIES, <Type, ExtensionDescriptor>{
  Gltf: ExtensionDescriptor(KhrRigidBodiesGltf.fromMap),
  Node: ExtensionDescriptor(KhrRigidBodiesNode.fromMap)
});

class RigidBodiesError extends IssueType {
  static final incorrectJointLimits = RigidBodiesError._(
      'KHR_RIGID_BODIES_JOINT_LIMITS_INCORRECT',
      (args) => 'Joint limit should contain either linear or angular axes');

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

  RigidBodiesError._(String type, ErrorFunction message,
      [Severity severity = Severity.Error])
      : super(type, message, severity);
}

const String PHYSICS_MATERIALS = 'physicsMaterials';
const String PHYSICS_JOINT_LIMITS = 'physicsJointLimits';
const String COLLISION_FILTERS = 'collisionFilters';
const String STATIC_FRICTION = 'staticFriction';
const String DYNAMIC_FRICTION = 'dynamicFriction';
const String RESTITUTION = 'restitution';
const String FRICTION_COMBINE = 'frictionCombine';
const String RESTITUTION_COMBINE = 'restitutionCombine';
const String AVERAGE = 'AVERAGE';
const String MINIMUM = 'MINIMUM';
const String MAXIMUM = 'MAXIMUM';
const String MULTIPLY = 'MULTIPLY';
const String COLLISION_SYSTEMS = 'collisionSystems';
const String COLLIDE_WITH_SYSTEMS = 'collideWithSystems';
const String NOT_COLLIDE_WITH_SYSTEMS = 'notCollideWithSystems';
const String COLLIDER = 'collider';
const String SHAPE = 'shape';
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
const String JOINT_LIMITS = 'jointLimits';
const String ENABLE_COLLISION = 'enableCollision';
const String SPRING_CONSTANT = 'springConstant';
const String SPRING_DAMPING = 'springDamping';
const String LINEAR_AXES = 'linearAxes';
const String ANGULAR_AXES = 'angularAxes';
const String LIMITS = 'limits';

const List<String> KHR_RIGID_BODIES_GLTF_MEMBERS = <String>[
  PHYSICS_MATERIALS,
  PHYSICS_JOINT_LIMITS,
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
const List<String> KHR_RIGID_BODIES_JOINT_LIMIT_SET_MEMBERS = <String>[LIMITS];
const List<String> KHR_RIGID_BODIES_JOINT_LIMIT_MEMBERS = <String>[
  MIN,
  MAX,
  SPRING_CONSTANT,
  SPRING_DAMPING,
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
const List<String> KHR_RIGID_BODIES_COLLIDER_MEMBERS = <String>[
  SHAPE,
  PHYSICS_MATERIAL,
  COLLISION_FILTER
];
const List<String> KHR_RIGID_BODIES_TRIGGER_MEMBERS = <String>[
  SHAPE,
  COLLISION_FILTER
];
const List<String> KHR_RIGID_BODIES_JOINT_MEMBERS = <String>[
  CONNECTED_NODE,
  JOINT_LIMITS,
  ENABLE_COLLISION
];

class KhrRigidBodiesGltf extends GltfProperty {
  final SafeList<KhrRigidBodiesPhysicsMaterial> physicsMaterials;
  final SafeList<KhrRigidBodiesCollisionFilter> collisionFilters;
  final SafeList<KhrRigidBodiesPhysicsJointLimitSet> jointLimits;

  KhrRigidBodiesGltf._(this.physicsMaterials, this.collisionFilters,
      this.jointLimits, Map<String, Object> extensions, Object extras)
      : super(extensions, extras);

  static KhrRigidBodiesGltf fromMap(Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, KHR_RIGID_BODIES_GLTF_MEMBERS, context);
    }

    var materials =
        SafeList<KhrRigidBodiesPhysicsMaterial>.empty(PHYSICS_MATERIALS);
    if (map.containsKey(PHYSICS_MATERIALS)) {
      final materialMaps = getMapList(map, PHYSICS_MATERIALS, context);
      if (materialMaps != null) {
        materials = SafeList<KhrRigidBodiesPhysicsMaterial>(
            materialMaps.length, PHYSICS_MATERIALS);
        context.path.add(PHYSICS_MATERIALS);
        for (var i = 0; i < materialMaps.length; i++) {
          context.path.add(i.toString());
          materials[i] =
              KhrRigidBodiesPhysicsMaterial.fromMap(materialMaps[i], context);
          context.path.removeLast();
        }
        context.path.removeLast();
      }
    }

    var jointLimits = SafeList<KhrRigidBodiesPhysicsJointLimitSet>.empty(
        PHYSICS_JOINT_LIMITS);
    if (map.containsKey(PHYSICS_JOINT_LIMITS)) {
      final jointLimitSetMaps = getMapList(map, PHYSICS_JOINT_LIMITS, context);
      if (jointLimitSetMaps != null) {
        jointLimits = SafeList<KhrRigidBodiesPhysicsJointLimitSet>(
            jointLimitSetMaps.length, PHYSICS_JOINT_LIMITS);
        context.path.add(PHYSICS_JOINT_LIMITS);
        for (var i = 0; i < jointLimitSetMaps.length; i++) {
          context.path.add(i.toString());
          jointLimits[i] = KhrRigidBodiesPhysicsJointLimitSet.fromMap(
              jointLimitSetMaps[i], context);
          context.path.removeLast();
        }
        context.path.removeLast();
      }
    }

    var filters =
        SafeList<KhrRigidBodiesCollisionFilter>.empty(COLLISION_FILTERS);
    if (map.containsKey(COLLISION_FILTERS)) {
      final filterListMaps = getMapList(map, COLLISION_FILTERS, context);
      if (filterListMaps != null) {
        filters = SafeList<KhrRigidBodiesCollisionFilter>(
            filterListMaps.length, COLLISION_FILTERS);
        context.path.add(COLLISION_FILTERS);
        for (var i = 0; i < filterListMaps.length; i++) {
          context.path.add(i.toString());
          filters[i] =
              KhrRigidBodiesCollisionFilter.fromMap(filterListMaps[i], context);
          context.path.removeLast();
        }
        context.path.removeLast();
      }
    }

    return KhrRigidBodiesGltf._(
        materials,
        filters,
        jointLimits,
        getExtensions(map, KhrRigidBodiesGltf, context),
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
    linkListItems(PHYSICS_JOINT_LIMITS, jointLimits);
  }
}

class KhrRigidBodiesNode extends GltfProperty {
  KhrRigidBodiesMotion motion;
  KhrRigidBodiesCollider collider;
  KhrRigidBodiesTrigger trigger;
  KhrRigidBodiesJoint joint;

  KhrRigidBodiesNode._(this.motion, this.collider, this.trigger, this.joint,
      Map<String, Object> extensions, Object extras)
      : super(extensions, extras);

  static KhrRigidBodiesNode fromMap(Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, KHR_RIGID_BODIES_NODE_MEMBERS, context);
    }

    final motion = getObjectFromInnerMap(
        map, MOTION, context, KhrRigidBodiesMotion.fromMap,
        req: false);
    final collider = getObjectFromInnerMap(
        map, COLLIDER, context, KhrRigidBodiesCollider.fromMap,
        req: false);
    final trigger = getObjectFromInnerMap(
        map, TRIGGER, context, KhrRigidBodiesTrigger.fromMap,
        req: false);
    final joint = getObjectFromInnerMap(
        map, JOINT, context, KhrRigidBodiesJoint.fromMap,
        req: false);

    return KhrRigidBodiesNode._(
        motion,
        collider,
        trigger,
        joint,
        getExtensions(map, KhrRigidBodiesNode, context),
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

class KhrRigidBodiesCollider extends GltfProperty {
  final int _shapeIndex;
  KhrCollisionShapesShape _shape;
  final int _physicsMaterialIndex;
  KhrRigidBodiesPhysicsMaterial _material;
  final int _collisionFilterIndex;
  KhrRigidBodiesCollisionFilter _collisionFilter;

  KhrRigidBodiesCollider._(this._shapeIndex, this._physicsMaterialIndex,
      this._collisionFilterIndex, Map<String, Object> extensions, Object extras)
      : super(extensions, extras);

  static KhrRigidBodiesCollider fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, KHR_RIGID_BODIES_COLLIDER_MEMBERS, context);
    }

    final shape = getIndex(map, SHAPE, context, req: true);
    final material = getIndex(map, PHYSICS_MATERIAL, context, req: false);
    final filter = getIndex(map, COLLISION_FILTER, context, req: false);
    return KhrRigidBodiesCollider._(
        shape,
        material,
        filter,
        getExtensions(map, KhrRigidBodiesCollider, context),
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

  KhrCollisionShapesShape get shape => _shape;
  KhrRigidBodiesPhysicsMaterial get material => _material;
  KhrRigidBodiesCollisionFilter get collisionFilter => _collisionFilter;
}

class KhrRigidBodiesTrigger extends GltfProperty {
  final int _shapeIndex;
  KhrCollisionShapesShape _shape;
  final int _collisionFilterIndex;
  KhrRigidBodiesCollisionFilter _collisionFilter;

  KhrRigidBodiesTrigger._(this._shapeIndex, this._collisionFilterIndex,
      Map<String, Object> extensions, Object extras)
      : super(extensions, extras);

  static KhrRigidBodiesTrigger fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, KHR_RIGID_BODIES_TRIGGER_MEMBERS, context);
    }

    final shape = getIndex(map, SHAPE, context, req: true);
    final filter = getIndex(map, COLLISION_FILTER, context, req: false);
    return KhrRigidBodiesTrigger._(
        shape,
        filter,
        getExtensions(map, KhrRigidBodiesTrigger, context),
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
    _collisionFilter =
        getCollisionFilterByIndex(gltf, context, _collisionFilterIndex);
    if (_collisionFilter != null) {
      context.path.add(COLLISION_FILTER);
      _collisionFilter.link(gltf, context);
      context.path.removeLast();
    }
  }

  KhrCollisionShapesShape get shape => _shape;
  KhrRigidBodiesCollisionFilter get collisionFilter => _collisionFilter;
}

KhrCollisionShapesShape getShapeByIndex(
    Gltf gltf, Context context, int shapeIndex) {
  final collisionShapesExtension =
      gltf.extensions[khrCollisionShapesExtension.name];
  if (collisionShapesExtension is KhrCollisionShapesGltf) {
    final shape = collisionShapesExtension.shapes[shapeIndex];
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
        args: ['/$EXTENSIONS/${khrCollisionShapesExtension.name}']);
  }
  return null;
}

KhrRigidBodiesCollisionFilter getCollisionFilterByIndex(
    Gltf gltf, Context context, int filterIndex) {
  final rigidBodiesExtension = gltf.extensions[khrRigidBodiesExtension.name];
  if (rigidBodiesExtension is KhrRigidBodiesGltf) {
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
        args: ['/$EXTENSIONS/${khrRigidBodiesExtension.name}']);
  }
  return null;
}

KhrRigidBodiesPhysicsMaterial getPhysicsMaterialByIndex(
    Gltf gltf, Context context, int materialIndex) {
  final rigidBodiesExtension = gltf.extensions[khrRigidBodiesExtension.name];
  if (rigidBodiesExtension is KhrRigidBodiesGltf) {
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
        args: ['/$EXTENSIONS/${khrRigidBodiesExtension.name}']);
  }
  return null;
}

class KhrRigidBodiesMotion extends GltfProperty {
  bool isKinematic;
  double mass;
  Vector3 centerOfMass;
  Vector3 inertiaDiagonal;
  Quaternion inertiaOrientation;
  Vector3 linearVelocity;
  Vector3 angularVelocity;
  double gravityFactor;

  KhrRigidBodiesMotion._(
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

  static KhrRigidBodiesMotion fromMap(
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

    return KhrRigidBodiesMotion._(
        isKinematic,
        mass,
        com,
        inertiaDiagonal,
        inertiaOrientation,
        linearVelocity,
        angularVelocity,
        gravityFactor,
        getExtensions(map, KhrRigidBodiesNode, context),
        getExtras(map, context));
  }

  static Vector3 _listToVec3(List<double> l) {
    if (l != null) {
      return Vector3(l[0], l[1], l[2]);
    }
    return null;
  }
}

class KhrRigidBodiesJoint extends GltfProperty {
  final int _connectedNodeIndex;
  Node _connectedNode;
  final int _jointLimitsIndex;
  KhrRigidBodiesPhysicsJointLimitSet _jointLimits;
  bool enableCollision;

  KhrRigidBodiesJoint._(this._connectedNodeIndex, this._jointLimitsIndex,
      this.enableCollision, Map<String, Object> extensions, Object extras)
      : super(extensions, extras);

  static KhrRigidBodiesJoint fromMap(Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, KHR_RIGID_BODIES_JOINT_MEMBERS, context);
    }

    final connectedNode = getIndex(map, CONNECTED_NODE, context, req: false);
    final jointLimits = getIndex(map, JOINT_LIMITS, context, req: true);
    final enableCollision = getBool(map, ENABLE_COLLISION, context);

    return KhrRigidBodiesJoint._(
        connectedNode,
        jointLimits,
        enableCollision,
        getExtensions(map, KhrRigidBodiesNode, context),
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

    final rigidBodiesExtension = gltf.extensions[khrRigidBodiesExtension.name];
    if (rigidBodiesExtension is KhrRigidBodiesGltf) {
      _jointLimits = rigidBodiesExtension.jointLimits[_jointLimitsIndex];
      if (_jointLimitsIndex != -1) {
        if (context.validate && _jointLimits == null) {
          context.addIssue(LinkError.unresolvedReference,
              name: JOINT_LIMITS, args: [_jointLimitsIndex]);
        } else if (_jointLimits != null) {
          _jointLimits.markAsUsed();
        }
      }
    } else if (context.validate && _jointLimitsIndex != -1) {
      context.addIssue(SchemaError.unsatisfiedDependency,
          args: ['/$EXTENSIONS/${khrRigidBodiesExtension.name}']);
    }
  }

  Node get connectedNode => _connectedNode;
  KhrRigidBodiesPhysicsJointLimitSet get jointLimits => _jointLimits;
}

class KhrRigidBodiesPhysicsMaterial extends GltfChildOfRootProperty {
  final double staticFriction;
  final double dynamicFriction;
  final double restitution;
  final String frictionCombine;
  final String restitutionCombine;

  KhrRigidBodiesPhysicsMaterial._(
      this.staticFriction,
      this.dynamicFriction,
      this.restitution,
      this.frictionCombine,
      this.restitutionCombine,
      String name,
      Map<String, Object> extensions,
      Object extras)
      : super(name, extensions, extras);

  static KhrRigidBodiesPhysicsMaterial fromMap(
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

    return KhrRigidBodiesPhysicsMaterial._(
        staticFriction,
        dynamicFriction,
        restitution,
        frictionCombine,
        restitutionCombine,
        getName(map, context),
        getExtensions(map, KhrRigidBodiesPhysicsMaterial, context),
        getExtras(map, context));
  }
}

class KhrRigidBodiesPhysicsJointLimit extends GltfProperty {
  double min;
  double max;
  double springConstant;
  double springDamping;
  List<int> linearAxes;
  List<int> angularAxes;
  KhrRigidBodiesPhysicsJointLimit._(
      this.min,
      this.max,
      this.springConstant,
      this.springDamping,
      this.linearAxes,
      this.angularAxes,
      Map<String, Object> extensions,
      Object extras)
      : super(extensions, extras);

  static KhrRigidBodiesPhysicsJointLimit fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, KHR_RIGID_BODIES_JOINT_LIMIT_MEMBERS, context);
    }

    final minLimit = getFloat(map, MIN, context);
    final maxLimit = getFloat(map, MAX, context);
    final springConstant = getFloat(map, SPRING_CONSTANT, context, min: 0);
    final springDamping = getFloat(map, SPRING_DAMPING, context, min: 0);
    final linLimits = getIndicesList(map, LINEAR_AXES, context);
    final angLimits = getIndicesList(map, ANGULAR_AXES, context);

    if (context.validate) {
      if (!((linLimits == null) ^ (angLimits == null))) {
        context.addIssue(RigidBodiesError.incorrectJointLimits);
      }

      if (minLimit > maxLimit) {
        context.addIssue(RigidBodiesError.incorrectJointRange,
            args: [minLimit, maxLimit]);
      }

      _validateLimitIndices(linLimits, LINEAR_AXES, context);
      _validateLimitIndices(angLimits, ANGULAR_AXES, context);
    }

    return KhrRigidBodiesPhysicsJointLimit._(
        minLimit,
        maxLimit,
        springConstant,
        springDamping,
        linLimits,
        angLimits,
        getExtensions(map, KhrRigidBodiesPhysicsJointLimit, context),
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

class KhrRigidBodiesPhysicsJointLimitSet extends GltfChildOfRootProperty {
  List<KhrRigidBodiesPhysicsJointLimit> limits;

  KhrRigidBodiesPhysicsJointLimitSet._(
      this.limits, String name, Map<String, Object> extensions, Object extras)
      : super(name, extensions, extras);

  static KhrRigidBodiesPhysicsJointLimitSet fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, KHR_RIGID_BODIES_JOINT_LIMIT_SET_MEMBERS, context);
    }

    SafeList<KhrRigidBodiesPhysicsJointLimit> limits;
    final jointLimitMaps = getMapList(map, LIMITS, context);
    if (jointLimitMaps != null) {
      limits = SafeList<KhrRigidBodiesPhysicsJointLimit>(
          jointLimitMaps.length, LIMITS);
      context.path.add(LIMITS);
      for (var i = 0; i < jointLimitMaps.length; i++) {
        context.path.add(i.toString());
        limits[i] =
            KhrRigidBodiesPhysicsJointLimit.fromMap(jointLimitMaps[i], context);
        context.path.removeLast();
      }
      context.path.removeLast();
    } else {
      limits = SafeList<KhrRigidBodiesPhysicsJointLimit>.empty(LIMITS);
    }

    return KhrRigidBodiesPhysicsJointLimitSet._(
        limits,
        getName(map, context),
        getExtensions(map, KhrRigidBodiesPhysicsJointLimitSet, context),
        getExtras(map, context));
  }
}

class KhrRigidBodiesCollisionFilter extends GltfChildOfRootProperty {
  SafeList<String> collisionSystems;
  SafeList<String> collideWithSystems;
  SafeList<String> notCollideWithSystems;

  KhrRigidBodiesCollisionFilter._(
      this.collisionSystems,
      this.collideWithSystems,
      this.notCollideWithSystems,
      String name,
      Map<String, Object> extensions,
      Object extras)
      : super(name, extensions, extras);

  static KhrRigidBodiesCollisionFilter fromMap(
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

    var collisionSystems = extractSystemList(map, COLLISION_SYSTEMS);
    var collideWithSystems = extractSystemList(map, COLLIDE_WITH_SYSTEMS);
    var notCollideWithSystems =
        extractSystemList(map, NOT_COLLIDE_WITH_SYSTEMS);

    return KhrRigidBodiesCollisionFilter._(
        collisionSystems,
        collideWithSystems,
        notCollideWithSystems,
        getName(map, context),
        getExtensions(map, KhrRigidBodiesCollisionFilter, context),
        getExtras(map, context));
  }
}
