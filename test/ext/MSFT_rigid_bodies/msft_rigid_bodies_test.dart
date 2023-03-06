import 'dart:async';

import 'package:gltf/src/ext/MSFT_collision_primitives/MSFT_collision_primitives.dart';
import 'package:gltf/src/ext/MSFT_rigid_bodies/MSFT_rigid_bodies.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math.dart';

import '../../utils.dart';

Future main() async {
  await compareReports('test/ext/MSFT_rigid_bodies');

  group('Evaluate valid objects', () {
    test('rigidbodies.MSFT_rigid_bodies', () async {
      final gltf = (await read(
              'ext/MSFT_rigid_bodies/data/rigidbodies/valid.gltf',
              ignoreUnused: true))
          .gltf;

      final rbExt = gltf.extensions['MSFT_rigid_bodies'] as MsftRigidBodiesGltf;

      final material = rbExt.physicsMaterials[0];
      expect(material.staticFriction, 0.25);
      expect(material.dynamicFriction, 0.5);
      expect(material.restitution, 1.0);
      expect(material.frictionCombine, 'MAXIMUM');
      expect(material.restitutionCombine, 'MULTIPLY');

      final jointLimits = rbExt.jointLimits[0].limits;
      expect(jointLimits[0].min, -1.0);
      expect(jointLimits[0].max, 1.0);
      expect(jointLimits[0].springConstant, 2.0);
      expect(jointLimits[0].springDamping, 4.0);
      expect(jointLimits[0].linearAxes, const [0, 1, 2]);
      expect(jointLimits[1].angularAxes, const [0, 1, 2]);
    });

    test('node.MSFT_rigid_bodies', () async {
      final gltf = (await read('ext/MSFT_rigid_bodies/data/node/valid.gltf',
              ignoreUnused: true))
          .gltf;

      final cpExt = gltf.extensions['MSFT_collision_primitives']
          as MsftCollisionPrimitivesGltf;
      final rbExt = gltf.extensions['MSFT_rigid_bodies'] as MsftRigidBodiesGltf;
      final rbNode =
          gltf.nodes[0].extensions['MSFT_rigid_bodies'] as MsftRigidBodiesNode;

      expect(rbNode.material, rbExt.physicsMaterials[0]);
      expect(rbNode.collider, cpExt.colliders[0]);
      expect(rbNode.rigidBody.isKinematic, true);
      expect(rbNode.rigidBody.centerOfMass, Vector3(0.25, 0.5, 1));
      // Quaternion does not have an operator==, so compare the storage.
      // https://github.com/google/vector_math.dart/pull/288
      expect(rbNode.rigidBody.inertiaOrientation.storage,
          const [0.0, 1.0, 0.0, 0.0]);
      expect(rbNode.rigidBody.inverseInertia, Vector3(0.25, 0.5, 1));
      expect(rbNode.rigidBody.linearVelocity, Vector3(1, 2, 4));
      expect(rbNode.rigidBody.angularVelocity, Vector3(-1, -2, -4));
      expect(rbNode.rigidBody.gravityFactor, -1.0);
      expect(rbNode.joint.jointLimits, rbExt.jointLimits[0]);
      expect(rbNode.joint.connectedNode, gltf.nodes[1]);
      expect(rbNode.joint.enableCollision, false);
    });
  });
}
