library gltf.extensions.msft_rigid_bodies;

import 'package:gltf/src/base/gltf_property.dart';
import 'package:gltf/src/ext/MSFT_collision_primitives/MSFT_collision_primitives.dart';
import 'package:gltf/src/ext/extensions.dart';
import 'package:vector_math/vector_math.dart';

const String MSFT_RIGID_BODIES = 'MSFT_rigid_bodies';

const Extension msftRigidBodiesExtension =
    Extension(MSFT_RIGID_BODIES, <Type, ExtensionDescriptor>{
  Gltf: ExtensionDescriptor(MsftRigidBodiesGltf.fromMap),
  Node: ExtensionDescriptor(MsftRigidBodiesNode.fromMap)
});

class RigidBodiesError extends IssueType {
  static final incorrectJointLimits = RigidBodiesError._(
      'MSFT_RIGID_BODIES_JOINT_LIMITS_INCORRECT',
      (args) => 'Joint limit should contain either linear or angular axes');

  static final incorrectJointRange = RigidBodiesError._(
      'MSFT_RIGID_BODIES_JOINT_INVALID_RANGE',
      (args) => 'Joint limit min (${args[0]}) should be less or equal to '
          'max (${args[1]})');

  static final incorrectLimitLength = RigidBodiesError._(
      'MSFT_RIGID_BODIES_JOINT_LIMITS_INCORRECT_SIZE',
      (args) => 'Length of limited axes should be in range [1,3)');

  static final limitIndexOutOfBounds = RigidBodiesError._(
      'MSFT_RIGID_BODIES_JOINT_LIMITS_INVALID_AXIS',
      (args) => 'Indices of limited axes should be in range [0,3)');

  RigidBodiesError._(String type, ErrorFunction message,
      [Severity severity = Severity.Error])
      : super(type, message, severity);
}

const String PHYSICS_MATERIALS = 'physicsMaterials';
const String PHYSICS_JOINT_LIMITS = 'physicsJointLimits';
const String STATIC_FRICTION = 'staticFriction';
const String DYNAMIC_FRICTION = 'dynamicFriction';
const String RESTITUTION = 'restitution';
const String FRICTION_COMBINE = 'frictionCombine';
const String RESTITUTION_COMBINE = 'restitutionCombine';
const String AVERAGE = 'AVERAGE';
const String MINIMUM = 'MINIMUM';
const String MAXIMUM = 'MAXIMUM';
const String MULTIPLY = 'MULTIPLY';
const String RIGID_BODY = 'rigidBody';
const String COLLIDER = 'collider';
const String PHYSICS_MATERIAL = 'physicsMaterial';
const String JOINT = 'joint';
const String IS_KINEMATIC = 'isKinematic';
const String INVERSE_MASS = 'inverseMass';
const String CENTER_OF_MASS = 'centerOfMass';
const String INERTIA_ORIENTATION = 'inertiaOrientation';
const String INVERSE_INERTIA_TENSOR = 'inverseInertiaTensor';
const String LINEAR_VELOCITY = 'linearVelocity';
const String ANGULAR_VELOCITY = 'angularVelocity';
const String GRAVITY_FACTOR = 'gravityFactor';
const String CONNECTED_NODE = 'connectedNode';
const String JOINT_LIMITS = 'jointLimits';
const String ENABLE_COLLISION = 'enableCollision';
const String COLLIDERS = 'colliders';
const String MIN = 'min';
const String MAX = 'max';
const String SPRING_CONSTANT = 'springConstant';
const String SPRING_DAMPING = 'springDamping';
const String LINEAR_AXES = 'linearAxes';
const String ANGULAR_AXES = 'angularAxes';
const String LIMITS = 'limits';

const List<String> MSFT_RIGID_BODIES_GLTF_MEMBERS = <String>[
  PHYSICS_MATERIALS,
  PHYSICS_JOINT_LIMITS
];
const List<String> MSFT_RIGID_BODIES_PHYSICS_MATERIAL_MEMBERS = <String>[
  STATIC_FRICTION,
  DYNAMIC_FRICTION,
  RESTITUTION,
  FRICTION_COMBINE,
  RESTITUTION_COMBINE
];
const List<String> MSFT_MATERIAL_COMBINE_MODES = <String>[
  AVERAGE,
  MINIMUM,
  MAXIMUM,
  MULTIPLY
];
const List<String> MSFT_RIGID_BODIES_NODE_MEMBERS = <String>[
  RIGID_BODY,
  COLLIDER,
  PHYSICS_MATERIAL,
  JOINT
];
const List<String> MSFT_RIGID_BODIES_RIGID_BODY_MEMBERS = <String>[
  IS_KINEMATIC,
  INVERSE_MASS,
  CENTER_OF_MASS,
  INERTIA_ORIENTATION,
  INVERSE_INERTIA_TENSOR,
  LINEAR_VELOCITY,
  ANGULAR_VELOCITY,
  GRAVITY_FACTOR
];
const List<String> MSFT_RIGID_BODIES_JOINT_MEMBERS = <String>[
  CONNECTED_NODE,
  JOINT_LIMITS,
  ENABLE_COLLISION
];
const List<String> MSFT_RIGID_BODIES_JOINT_LIMIT_MEMBERS = <String>[
  MIN,
  MAX,
  SPRING_CONSTANT,
  SPRING_DAMPING,
  LINEAR_AXES,
  ANGULAR_AXES
];
const List<String> MSFT_RIGID_BODIES_JOINT_LIMIT_SET_MEMBERS = <String>[LIMITS];

class MsftRigidBodiesGltf extends GltfProperty {
  final SafeList<MsftRigidBodiesPhysicsMaterial> physicsMaterials;
  final SafeList<MsftRigidBodiesPhysicsJointLimitSet> jointLimits;

  MsftRigidBodiesGltf._(this.physicsMaterials, this.jointLimits,
      Map<String, Object> extensions, Object extras)
      : super(extensions, extras);

  static MsftRigidBodiesGltf fromMap(Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, MSFT_RIGID_BODIES_GLTF_MEMBERS, context);
    }

    var materials =
        SafeList<MsftRigidBodiesPhysicsMaterial>.empty(PHYSICS_MATERIALS);
    if (map.containsKey(PHYSICS_MATERIALS)) {
      final materialMaps = getMapList(map, PHYSICS_MATERIALS, context);
      if (materialMaps != null) {
        materials = SafeList<MsftRigidBodiesPhysicsMaterial>(
            materialMaps.length, PHYSICS_MATERIALS);
        context.path.add(PHYSICS_MATERIALS);
        for (var i = 0; i < materialMaps.length; i++) {
          context.path.add(i.toString());
          materials[i] =
              MsftRigidBodiesPhysicsMaterial.fromMap(materialMaps[i], context);
          context.path.removeLast();
        }
        context.path.removeLast();
      }
    }

    var jointLimits = SafeList<MsftRigidBodiesPhysicsJointLimitSet>.empty(
        PHYSICS_JOINT_LIMITS);
    if (map.containsKey(PHYSICS_JOINT_LIMITS)) {
      final jointLimitSetMaps = getMapList(map, PHYSICS_JOINT_LIMITS, context);
      if (jointLimitSetMaps != null) {
        jointLimits = SafeList<MsftRigidBodiesPhysicsJointLimitSet>(
            jointLimitSetMaps.length, PHYSICS_JOINT_LIMITS);
        context.path.add(PHYSICS_JOINT_LIMITS);
        for (var i = 0; i < jointLimitSetMaps.length; i++) {
          context.path.add(i.toString());
          jointLimits[i] = MsftRigidBodiesPhysicsJointLimitSet.fromMap(
              jointLimitSetMaps[i], context);
          context.path.removeLast();
        }
        context.path.removeLast();
      }
    }

    return MsftRigidBodiesGltf._(
        materials,
        jointLimits,
        getExtensions(map, MsftRigidBodiesGltf, context),
        getExtras(map, context));
  }

  @override
  void link(Gltf gltf, Context context) {
    context.path.add(PHYSICS_MATERIALS);
    context.extensionCollections[physicsMaterials] =
        context.path.toList(growable: false);
    physicsMaterials.forEachWithIndices((i, material) {
      context.path.add(i.toString());
      material.link(gltf, context);
      context.path.removeLast();
    });
    context.path.removeLast();

    context.path.add(PHYSICS_JOINT_LIMITS);
    context.extensionCollections[jointLimits] =
        context.path.toList(growable: false);
    jointLimits.forEachWithIndices((i, limitSet) {
      context.path.add(i.toString());
      limitSet.link(gltf, context);
      context.path.removeLast();
    });
    context.path.removeLast();
  }
}

class MsftRigidBodiesNode extends GltfProperty {
  MsftRigidBodiesRigidBody rigidBody;
  final int _colliderIndex;
  MsftCollisionPrimitivesCollider _collider;
  final int _physicsMaterialIndex;
  MsftRigidBodiesPhysicsMaterial _material;
  MsftRigidBodiesJoint joint;

  MsftRigidBodiesNode._(
      this.rigidBody,
      this._colliderIndex,
      this._physicsMaterialIndex,
      this.joint,
      Map<String, Object> extensions,
      Object extras)
      : super(extensions, extras);

  static MsftRigidBodiesNode fromMap(Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, MSFT_RIGID_BODIES_NODE_MEMBERS, context);
    }

    final rigidBody = getObjectFromInnerMap(
        map, RIGID_BODY, context, MsftRigidBodiesRigidBody.fromMap,
        req: false);
    final collider = getIndex(map, COLLIDER, context, req: false);
    final material = getIndex(map, PHYSICS_MATERIAL, context, req: false);
    final joint = getObjectFromInnerMap(
        map, JOINT, context, MsftRigidBodiesJoint.fromMap,
        req: false);

    return MsftRigidBodiesNode._(
        rigidBody,
        collider,
        material,
        joint,
        getExtensions(map, MsftRigidBodiesNode, context),
        getExtras(map, context));
  }

  @override
  void link(Gltf gltf, Context context) {
    final collisionPrimitivesExtension =
        gltf.extensions[msftCollisionPrimitivesExtension.name];
    if (collisionPrimitivesExtension is MsftCollisionPrimitivesGltf) {
      _collider = collisionPrimitivesExtension.colliders[_colliderIndex];
      if (_colliderIndex != -1) {
        if (context.validate && _collider == null) {
          context.addIssue(LinkError.unresolvedReference,
              name: COLLIDER, args: [_colliderIndex]);
        } else if (_collider != null) {
          _collider.markAsUsed();
        }
      }
    } else if (context.validate && _colliderIndex >= 0) {
      context.addIssue(SchemaError.unsatisfiedDependency,
          args: ['/$EXTENSIONS/${msftCollisionPrimitivesExtension.name}']);
    }

    final rigidBodiesExtension = gltf.extensions[msftRigidBodiesExtension.name];
    if (rigidBodiesExtension is MsftRigidBodiesGltf) {
      _material = rigidBodiesExtension.physicsMaterials[_physicsMaterialIndex];
      if (_physicsMaterialIndex != -1) {
        if (context.validate && _material == null) {
          context.addIssue(LinkError.unresolvedReference,
              name: PHYSICS_MATERIAL, args: [_physicsMaterialIndex]);
        } else if (material != null) {
          _material.markAsUsed();
        }
      }
    } else if (context.validate && _physicsMaterialIndex >= 0) {
      context.addIssue(SchemaError.unsatisfiedDependency,
          args: ['/$EXTENSIONS/${msftRigidBodiesExtension.name}']);
    }

    if (rigidBody != null) {
      context.path.add(RIGID_BODY);
      rigidBody.link(gltf, context);
      context.path.removeLast();
    }

    if (joint != null) {
      context.path.add(JOINT);
      joint.link(gltf, context);
      context.path.removeLast();
    }
  }

  MsftCollisionPrimitivesCollider get collider => _collider;
  MsftRigidBodiesPhysicsMaterial get material => _material;
}

class MsftRigidBodiesRigidBody extends GltfProperty {
  bool isKinematic;
  double inverseMass;
  Vector3 centerOfMass;
  Quaternion inertiaOrientation;
  Vector3 inverseInertia;
  Vector3 linearVelocity;
  Vector3 angularVelocity;
  double gravityFactor;

  MsftRigidBodiesRigidBody._(
      this.isKinematic,
      this.inverseMass,
      this.centerOfMass,
      this.inertiaOrientation,
      this.inverseInertia,
      this.linearVelocity,
      this.angularVelocity,
      this.gravityFactor,
      Map<String, Object> extensions,
      Object extras)
      : super(extensions, extras);

  static MsftRigidBodiesRigidBody fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, MSFT_RIGID_BODIES_RIGID_BODY_MEMBERS, context);
    }

    final isKinematic = getBool(map, IS_KINEMATIC, context);
    final inverseMass =
        getFloat(map, INVERSE_MASS, context, req: false, def: 1, min: 0);

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

    final inverseInertiaList = getFloatList(
        map, INVERSE_INERTIA_TENSOR, context,
        min: 0, lengthsList: const [3], def: [1.0, 1.0, 1.0]);
    final inverseInertia = _listToVec3(inverseInertiaList);

    final linearVelocityList = getFloatList(map, LINEAR_VELOCITY, context,
        lengthsList: const [3], def: [0.0, 0.0, 0.0]);
    final linearVelocity = _listToVec3(linearVelocityList);

    final angularVelocityList = getFloatList(map, ANGULAR_VELOCITY, context,
        lengthsList: const [3], def: [0.0, 0.0, 0.0]);
    final angularVelocity = _listToVec3(angularVelocityList);

    final gravityFactor =
        getFloat(map, GRAVITY_FACTOR, context, req: false, def: 1);

    return MsftRigidBodiesRigidBody._(
        isKinematic,
        inverseMass,
        com,
        inertiaOrientation,
        inverseInertia,
        linearVelocity,
        angularVelocity,
        gravityFactor,
        getExtensions(map, MsftRigidBodiesNode, context),
        getExtras(map, context));
  }

  static Vector3 _listToVec3(List<double> l) {
    if (l != null) {
      return Vector3(l[0], l[1], l[2]);
    }
    return null;
  }
}

class MsftRigidBodiesJoint extends GltfProperty {
  final int _connectedNodeIndex;
  Node _connectedNode;
  final int _jointLimitsIndex;
  MsftRigidBodiesPhysicsJointLimitSet _jointLimits;
  bool enableCollision;

  MsftRigidBodiesJoint._(this._connectedNodeIndex, this._jointLimitsIndex,
      this.enableCollision, Map<String, Object> extensions, Object extras)
      : super(extensions, extras);

  static MsftRigidBodiesJoint fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, MSFT_RIGID_BODIES_JOINT_MEMBERS, context);
    }

    final connectedNode = getIndex(map, CONNECTED_NODE, context, req: false);
    final jointLimits = getIndex(map, JOINT_LIMITS, context, req: true);
    final enableCollision = getBool(map, ENABLE_COLLISION, context);

    return MsftRigidBodiesJoint._(
        connectedNode,
        jointLimits,
        enableCollision,
        getExtensions(map, MsftRigidBodiesNode, context),
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

    final rigidBodiesExtension = gltf.extensions[msftRigidBodiesExtension.name];
    if (rigidBodiesExtension is MsftRigidBodiesGltf) {
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
          args: ['/$EXTENSIONS/${msftRigidBodiesExtension.name}']);
    }
  }

  Node get connectedNode => _connectedNode;
  MsftRigidBodiesPhysicsJointLimitSet get jointLimits => _jointLimits;
}

class MsftRigidBodiesPhysicsMaterial extends GltfChildOfRootProperty {
  final double staticFriction;
  final double dynamicFriction;
  final double restitution;
  final String frictionCombine;
  final String restitutionCombine;

  MsftRigidBodiesPhysicsMaterial._(
      this.staticFriction,
      this.dynamicFriction,
      this.restitution,
      this.frictionCombine,
      this.restitutionCombine,
      String name,
      Map<String, Object> extensions,
      Object extras)
      : super(name, extensions, extras);

  static MsftRigidBodiesPhysicsMaterial fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, MSFT_RIGID_BODIES_PHYSICS_MATERIAL_MEMBERS, context);
    }

    final staticFriction =
        getFloat(map, STATIC_FRICTION, context, min: 0, def: 0.6);
    final dynamicFriction =
        getFloat(map, DYNAMIC_FRICTION, context, min: 0, def: 0.6);
    final restitution = getFloat(map, RESTITUTION, context, min: 0, def: 0.6);
    final frictionCombine = getString(map, FRICTION_COMBINE, context,
        list: MSFT_MATERIAL_COMBINE_MODES);
    final restitutionCombine = getString(map, RESTITUTION_COMBINE, context,
        list: MSFT_MATERIAL_COMBINE_MODES);

    return MsftRigidBodiesPhysicsMaterial._(
        staticFriction,
        dynamicFriction,
        restitution,
        frictionCombine,
        restitutionCombine,
        getName(map, context),
        getExtensions(map, MsftRigidBodiesPhysicsMaterial, context),
        getExtras(map, context));
  }
}

class MsftRigidBodiesPhysicsJointLimit extends GltfProperty {
  double min;
  double max;
  double springConstant;
  double springDamping;
  List<int> linearAxes;
  List<int> angularAxes;
  MsftRigidBodiesPhysicsJointLimit._(
      this.min,
      this.max,
      this.springConstant,
      this.springDamping,
      this.linearAxes,
      this.angularAxes,
      Map<String, Object> extensions,
      Object extras)
      : super(extensions, extras);

  static MsftRigidBodiesPhysicsJointLimit fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, MSFT_RIGID_BODIES_JOINT_LIMIT_MEMBERS, context);
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

    return MsftRigidBodiesPhysicsJointLimit._(
        minLimit,
        maxLimit,
        springConstant,
        springDamping,
        linLimits,
        angLimits,
        getExtensions(map, MsftRigidBodiesPhysicsJointLimit, context),
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

class MsftRigidBodiesPhysicsJointLimitSet extends GltfChildOfRootProperty {
  List<MsftRigidBodiesPhysicsJointLimit> limits;

  MsftRigidBodiesPhysicsJointLimitSet._(
      this.limits, String name, Map<String, Object> extensions, Object extras)
      : super(name, extensions, extras);

  static MsftRigidBodiesPhysicsJointLimitSet fromMap(
      Map<String, Object> map, Context context) {
    if (context.validate) {
      checkMembers(map, MSFT_RIGID_BODIES_JOINT_LIMIT_SET_MEMBERS, context);
    }

    SafeList<MsftRigidBodiesPhysicsJointLimit> limits;
    final jointLimitMaps = getMapList(map, LIMITS, context);
    if (jointLimitMaps != null) {
      limits = SafeList<MsftRigidBodiesPhysicsJointLimit>(
          jointLimitMaps.length, LIMITS);
      context.path.add(LIMITS);
      for (var i = 0; i < jointLimitMaps.length; i++) {
        context.path.add(i.toString());
        limits[i] = MsftRigidBodiesPhysicsJointLimit.fromMap(
            jointLimitMaps[i], context);
        context.path.removeLast();
      }
      context.path.removeLast();
    } else {
      limits = SafeList<MsftRigidBodiesPhysicsJointLimit>.empty(LIMITS);
    }

    return MsftRigidBodiesPhysicsJointLimitSet._(
        limits,
        getName(map, context),
        getExtensions(map, MsftRigidBodiesPhysicsJointLimitSet, context),
        getExtras(map, context));
  }
}
