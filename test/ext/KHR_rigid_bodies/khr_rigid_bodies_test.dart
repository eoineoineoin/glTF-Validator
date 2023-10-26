import 'dart:async';

import 'package:gltf/src/ext/KHR_collision_shapes/KHR_collision_shapes.dart';
import 'package:gltf/src/ext/KHR_rigid_bodies/KHR_rigid_bodies.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math.dart';

import '../../utils.dart';

Future main() async {
  await compareReports('test/ext/KHR_rigid_bodies');

  group('Evaluate valid objects', () {
    test('rigidbodies.KHR_rigid_bodies', () async {
      final gltf = (await read(
              'ext/KHR_rigid_bodies/data/rigidbodies/valid.gltf',
              ignoreUnused: true))
          .gltf;

      final rbExt = gltf.extensions['KHR_rigid_bodies'] as KhrRigidBodiesGltf;

      final material = rbExt.physicsMaterials[0];
      expect(material.staticFriction, 0.25);
      expect(material.dynamicFriction, 0.5);
      expect(material.restitution, 1.0);
      expect(material.frictionCombine, 'maximum');
      expect(material.restitutionCombine, 'multiply');

      final joint = rbExt.jointDefinitions[0];
      final jointLimits = joint.limits;
      expect(jointLimits[0].min, -1.0);
      expect(jointLimits[0].max, 1.0);
      expect(jointLimits[0].springConstant, 2.0);
      expect(jointLimits[0].springDamping, 4.0);
      expect(jointLimits[0].linearAxes, const [0, 1, 2]);
      expect(jointLimits[1].angularAxes, const [0, 1, 2]);
      final jointDrive = joint.drives[0];
      expect(jointDrive.type, 'linear');
      expect(jointDrive.mode, 'force');
      expect(jointDrive.axis, 0);
      expect(jointDrive.maxForce, 1.0);
      expect(jointDrive.velocityTarget, 2);
      expect(jointDrive.damping, 3);
      expect(jointDrive.positionTarget, 4);
      expect(jointDrive.stiffness, 5);
    });

    test('node.KHR_rigid_bodies', () async {
      final gltf = (await read('ext/KHR_rigid_bodies/data/node/valid.gltf',
              ignoreUnused: true))
          .gltf;

      final csExt =
          gltf.extensions['KHR_collision_shapes'] as KhrCollisionShapesGltf;
      final rbExt = gltf.extensions['KHR_rigid_bodies'] as KhrRigidBodiesGltf;
      final rbNode =
          gltf.nodes[0].extensions['KHR_rigid_bodies'] as KhrRigidBodiesNode;

      expect(rbNode.collider.material, rbExt.physicsMaterials[0]);
      expect(rbNode.collider.shape, csExt.shapes[0]);
      expect(rbNode.motion.isKinematic, true);
      expect(rbNode.motion.centerOfMass, Vector3(0.25, 0.5, 1));
      // Quaternion does not have an operator==, so compare the storage.
      // https://github.com/google/vector_math.dart/pull/288
      expect(
          rbNode.motion.inertiaOrientation.storage, const [0.0, 1.0, 0.0, 0.0]);
      expect(rbNode.motion.inertiaDiagonal, Vector3(0.25, 0.5, 1));
      expect(rbNode.motion.linearVelocity, Vector3(1, 2, 4));
      expect(rbNode.motion.angularVelocity, Vector3(-1, -2, -4));
      expect(rbNode.motion.gravityFactor, -1.0);
      expect(rbNode.joint.jointDefinition, rbExt.jointDefinitions[0]);
      expect(rbNode.joint.connectedNode, gltf.nodes[1]);
      expect(rbNode.joint.enableCollision, false);
    });
  });
}
