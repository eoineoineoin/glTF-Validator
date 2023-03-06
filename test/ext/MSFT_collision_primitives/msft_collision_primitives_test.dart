import 'dart:async';

import 'package:gltf/src/ext/MSFT_collision_primitives/MSFT_collision_primitives.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math.dart';

import '../../utils.dart';

Future main() async {
  await compareReports('test/ext/MSFT_collision_primitives');

  group('Evaluate valid objects', () {
    test('colliders.MSFT_collision_primitives', () async {
      final gltf = (await read(
              'ext/MSFT_collision_primitives/data/colliders/valid.gltf',
              ignoreUnused: true))
          .gltf;

      final collisionPrimitives = gltf.extensions['MSFT_collision_primitives']
          as MsftCollisionPrimitivesGltf;

      final sphere = collisionPrimitives.colliders[0].geom
          as MsftCollisionPrimitivesColliderSphere;
      expect(sphere, collisionPrimitives.colliders[0].geom);
      expect(sphere.radius, 0.5);

      final box = collisionPrimitives.colliders[1].geom
          as MsftCollisionPrimitivesColliderBox;
      expect(box, collisionPrimitives.colliders[1].geom);
      expect(box.size, Vector3(0.25, 0.5, 1));

      final capsule = collisionPrimitives.colliders[2].geom
          as MsftCollisionPrimitivesColliderCapsule;
      expect(capsule, collisionPrimitives.colliders[2].geom);
      expect(capsule.height, 2.0);
      expect(capsule.radius, 0.5);

      final cylinder = collisionPrimitives.colliders[3].geom
          as MsftCollisionPrimitivesColliderCylinder;
      expect(cylinder, collisionPrimitives.colliders[3].geom);
      expect(cylinder.height, 1.0);
      expect(cylinder.radius, 0.5);

      final convex = collisionPrimitives.colliders[4].geom
          as MsftCollisionPrimitivesColliderConvex;
      expect(convex, collisionPrimitives.colliders[4].geom);
      expect(convex.mesh, isNotNull);

      final trimesh = collisionPrimitives.colliders[5].geom
          as MsftCollisionPrimitivesColliderTrimesh;
      expect(trimesh, collisionPrimitives.colliders[5].geom);
      expect(trimesh.mesh, isNotNull);
    });
  });
}
