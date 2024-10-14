import 'dart:async';

import 'package:gltf/src/ext/KHR_implicit_shapes/KHR_implicit_shapes.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math.dart';

import '../../utils.dart';

Future main() async {
  await compareReports('test/ext/KHR_implicit_shapes');

  group('Evaluate valid objects', () {
    test('shapes.KHR_implicit_shapes', () async {
      final gltf = (await read('ext/KHR_implicit_shapes/data/shapes/valid.gltf',
              ignoreUnused: true))
          .gltf;

      final implicitShapes =
          gltf.extensions['KHR_implicit_shapes'] as KhrImplicitShapesGltf;

      final sphere =
          implicitShapes.shapes[0].geom as KhrImplicitShapesShapeSphere;
      expect(sphere, implicitShapes.shapes[0].geom);
      expect(sphere.radius, 0.5);

      final box = implicitShapes.shapes[1].geom as KhrImplicitShapesShapeBox;
      expect(box, implicitShapes.shapes[1].geom);
      expect(box.size, Vector3(0.25, 0.5, 1));

      final capsule =
          implicitShapes.shapes[2].geom as KhrImplicitShapesShapeCapsule;
      expect(capsule, implicitShapes.shapes[2].geom);
      expect(capsule.height, 2.0);
      expect(capsule.radiusBottom, 0.5);
      expect(capsule.radiusTop, 0.5);

      final cylinder =
          implicitShapes.shapes[3].geom as KhrImplicitShapesShapeCylinder;
      expect(cylinder, implicitShapes.shapes[3].geom);
      expect(cylinder.height, 1.0);
      expect(cylinder.radiusBottom, 0.5);
      expect(cylinder.radiusTop, 0.5);
    });
  });
}
