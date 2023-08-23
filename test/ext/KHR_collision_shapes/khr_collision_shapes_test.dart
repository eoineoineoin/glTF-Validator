import 'dart:async';

import 'package:gltf/src/ext/KHR_collision_shapes/KHR_collision_shapes.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math.dart';

import '../../utils.dart';

Future main() async {
  await compareReports('test/ext/KHR_collision_shapes');

  group('Evaluate valid objects', () {
    test('shapes.KHR_collision_shapes', () async {
      final gltf = (await read(
              'ext/KHR_collision_shapes/data/shapes/valid.gltf',
              ignoreUnused: true))
          .gltf;

      final collisionShapes =
          gltf.extensions['KHR_collision_shapes'] as KhrCollisionShapesGltf;

      final sphere =
          collisionShapes.shapes[0].geom as KhrCollisionShapesShapeSphere;
      expect(sphere, collisionShapes.shapes[0].geom);
      expect(sphere.radius, 0.5);

      final box = collisionShapes.shapes[1].geom as KhrCollisionShapesShapeBox;
      expect(box, collisionShapes.shapes[1].geom);
      expect(box.size, Vector3(0.25, 0.5, 1));

      final capsule =
          collisionShapes.shapes[2].geom as KhrCollisionShapesShapeCapsule;
      expect(capsule, collisionShapes.shapes[2].geom);
      expect(capsule.height, 2.0);
      expect(capsule.radiusBottom, 0.5);
      expect(capsule.radiusTop, 0.5);

      final cylinder =
          collisionShapes.shapes[3].geom as KhrCollisionShapesShapeCylinder;
      expect(cylinder, collisionShapes.shapes[3].geom);
      expect(cylinder.height, 1.0);
      expect(cylinder.radiusBottom, 0.5);
      expect(cylinder.radiusTop, 0.5);

      final convex =
          collisionShapes.shapes[4].geom as KhrCollisionShapesShapeConvex;
      expect(convex, collisionShapes.shapes[4].geom);
      expect(convex.mesh, isNotNull);

      final trimesh =
          collisionShapes.shapes[5].geom as KhrCollisionShapesShapeTrimesh;
      expect(trimesh, collisionShapes.shapes[5].geom);
      expect(trimesh.mesh, isNotNull);
    });
  });
}
