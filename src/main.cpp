#include "collision.hpp"
#include "consts.hpp"
#include "cuboid.hpp"
#include "joints/force.hpp"
#include "joints/spring.hpp"
#include "material.hpp"
#include "object.hpp"
#include "polyhedron.hpp"
#include <Camera3D.hpp>
#include <Color.hpp>
#include <Vector3.hpp>
#include <Window.hpp>
#include <memory>
#include <print>
#include <raylib.h>
#include <rlgl.h>
#include <vector>

int main() {
  const int screenWidth = 800;
  const int screenHeight = 450;

  raylib::Window window(
      screenWidth,
      screenHeight,
      "UMM Car simulator",
      FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT
  );

  rlEnableBackfaceCulling();

  // Initialize the camera
  raylib::Camera3D camera{
      {30.0f, 20.0f, 30.0f},
      {0.0f, 0.0f, 0.0f},
      {0.0f, 1.0f, 0.0f},
      70.0f,
      CAMERA_PERSPECTIVE
  };

  const BodyMaterial lubricated{0.05f, 0.005f};
  const BodyMaterial steel{0.8f, 0.6f};
  const BodyMaterial rubber{1.0f, 0.85f};

  std::vector<std::shared_ptr<Object>> game_objects;

  // std::shared_ptr<Body> cube = std::make_shared<Cuboid>(
  //     raylib::Vector3{0.4f, 1.0f, 0.1f},
  //     raylib::Color::Red(),
  //     rubber,
  //     raylib::Vector3{0.0f, 8.0f, 0.0f}
  // );
  // cube->mass(1.f);

  std::shared_ptr<Body> cube = std::make_shared<Polyhedron>(
      raylib::Mesh::Cube(0.4f, 1.0f, 0.1f),
      7500.f,
      raylib::Color::Red(),
      rubber,
      raylib::Vector3{0.0f, 8.0f, 0.0f}
  );

  game_objects.push_back(cube);

  cube->apply_force({0.0f, 0.0f, 10.f}, raylib::Vector3{0.1f, 3.0f, 0.0f});

  // std::shared_ptr<Body> cube1 = std::make_shared<Cuboid>(
  //     raylib::Vector3{2.0f, 2.0f, 2.0f},
  //     raylib::Color::Red(),
  //     rubber,
  //     raylib::Vector3{5.0f, 1.0f, 0.0f}
  // );
  //
  // cube1->mass(1000.f);
  //
  // game_objects.push_back(cube1);
  //
  // std::shared_ptr<Body> cube2 = std::make_shared<Cuboid>(
  //     raylib::Vector3{2.0f, 2.f, 6.0f},
  //     raylib::Color::Blue(),
  //     lubricated,
  //     raylib::Vector3{-5.0f, 1.0f, 0.0f}
  // );
  //
  // cube2->mass(1000.f);
  //
  // game_objects.push_back(cube2);
  //
  // std::shared_ptr<Body> cube3 = std::make_shared<Polyhedron>(
  //     raylib::Mesh::Cube(2.0f, 2.0f, 2.0f),
  //     150.f,
  //     raylib::Color::Yellow(),
  //     steel,
  //     raylib::Vector3{5.0f, 1.0f, -9.0f}
  // );
  //
  // game_objects.push_back(cube3);
  //
  // std::shared_ptr<Body> cube4 = std::make_shared<Polyhedron>(
  //     raylib::Mesh::Cube(2.0f, 2.0f, 2.0f),
  //     100.f,
  //     raylib::Color::Black(),
  //     steel,
  //     raylib::Vector3{0.0f, 1.0f, 8.0f}
  // );
  //
  // game_objects.push_back(cube4);
  //
  // std::shared_ptr<Joint> joint = std::make_shared<SpringJoint>(
  //     std::static_pointer_cast<Body>(cube1),
  //     std::static_pointer_cast<Body>(cube2),
  //     raylib::Color::Green(),
  //     1000.f,
  //     8.f,
  //     0.4f
  // );
  //
  // joint->anchor_a(raylib::Vector3{-1.0f, .0f, 1.0f})
  //     .anchor_b(raylib::Vector3{1.0f, .0f, 1.0f});
  //
  // std::shared_ptr<Joint> joint2 = std::make_shared<SpringJoint>(
  //     std::static_pointer_cast<Body>(cube1),
  //     std::static_pointer_cast<Body>(cube3),
  //     raylib::Color::Green(),
  //     1000.f,
  //     8.f,
  //     0.4f
  // );
  //
  // joint2->anchor_a(raylib::Vector3{1.0f, .0f, -1.0f})
  //     .anchor_b(raylib::Vector3{1.0f, .0f, 1.0f});
  //
  // std::shared_ptr<Joint> joint3 = std::make_shared<SpringJoint>(
  //     std::static_pointer_cast<Body>(cube1),
  //     std::static_pointer_cast<Body>(cube4),
  //     raylib::Color::Green(),
  //     1000.f,
  //     8.f,
  //     0.4f
  // );
  //
  // joint3->anchor_a(raylib::Vector3{1.0f, .0f, 1.0f})
  //     .anchor_b(raylib::Vector3{-1.0f, .0f, -1.0f});
  //
  // game_objects.push_back(joint);
  // game_objects.push_back(joint2);
  // game_objects.push_back(joint3);

  // std::shared_ptr<Polyhedron> polyhedron1 = std::make_shared<Polyhedron>(
  //     raylib::Mesh::Cube(2.0f, 2.0f, 2.0f),
  //     1000.f,
  //     raylib::Color::Red(),
  //     steel,
  //     raylib::Vector3{0.0f, 1.0f, 0.0f}
  // );
  //
  // game_objects.push_back(polyhedron1);
  //
  // std::shared_ptr<Polyhedron> polyhedron2 = std::make_shared<Polyhedron>(
  //     raylib::Mesh::Cube(2.0f, 2.0f, 2.0f),
  //     1000.f,
  //     raylib::Color::Blue(),
  //     steel,
  //     raylib::Vector3{2.0f, 1.0f, 0.0f},
  //     raylib::Quaternion::FromAxisAngle(
  //         raylib::Vector3{0.0f, 1.0f, 0.0f}, PI / 6
  //     ) *
  //         raylib::Quaternion::FromAxisAngle(
  //             raylib::Vector3{0.0f, 0.0f, 1.0f}, PI / 6
  //         )
  // );
  //
  // game_objects.push_back(polyhedron2);

  for (auto &object : game_objects) {
    object->initialize();
  }
  //--------------------------------------------------------------------------------------

  // Physics step time in seconds
  constexpr float STEP_TIME = 1.e-2f;
  double physics_time = 0.0f;

  // Main game loop
  while (!window.ShouldClose()) // Detect window close button or ESC key
  {
    // Update
    //----------------------------------------------------------------------------------
    const auto simulation_time = window.GetTime();

    while (physics_time < simulation_time) {
      std::println(
          "\n\nStepping physics... {} -> {}",
          physics_time,
          physics_time + STEP_TIME
      );
      physics_time += STEP_TIME;

      // Apply gravity
      // cube1->apply_force(GRAVITY * cube1->mass());
      // cube2->apply_force(GRAVITY * cube2->mass());
      // cube3->apply_force(GRAVITY * cube3->mass());

      // Update all alive objects
      for (const auto &object : game_objects) {
        if (object->alive) {
          object->update(STEP_TIME);
        }
      }

      // const auto collision =
      //     detect_polyhedron_collision(polyhedron1, polyhedron2);
      //
      // if (collision.has_value()) {
      //   const auto &value = collision.value();
      //   std::println(
      //       "collision: normal: {}, point: {}, depth: {}",
      //       value.normal,
      //       value.penetration_point,
      //       value.depth
      //   );
      // }

      // Remove dead objects
      std::erase_if(game_objects, [](const auto &object) {
        return !object->alive;
      });
    }

    // const auto move_force = 10000.f;
    //
    // if (IsKeyDown(KEY_SPACE)) {
    //   cube1->apply_force(-GRAVITY * 1500.f);
    // }
    //
    // if (IsKeyDown(KEY_W)) {
    //   cube1->apply_force(raylib::Vector3{0.0f, 0.0f, -move_force});
    // }
    //
    // if (IsKeyDown(KEY_S)) {
    //   cube1->apply_force(raylib::Vector3{0.0f, 0.0f, move_force});
    // }
    //
    // if (IsKeyDown(KEY_A)) {
    //   cube1->apply_force(raylib::Vector3{-move_force, 0.0f, 0.0f});
    // }
    //
    // if (IsKeyDown(KEY_D)) {
    //   cube1->apply_force(raylib::Vector3{move_force, 0.0f, 0.0f});
    // }
    //
    // camera.target = (cube1->position() + cube2->position()) / 2.f;
    camera.target = cube->position();
    camera.Update(CAMERA_THIRD_PERSON);
    //----------------------------------------------------------------------------------

    // Draw
    //----------------------------------------------------------------------------------
    window.BeginDrawing();

    window.ClearBackground(raylib::Color::RayWhite());

    camera.BeginMode();

    DrawGrid(500, 1.0f);

    for (const auto &object : game_objects) {
      object->draw();
    }

    camera.EndMode();

    window.DrawFPS();

    window.EndDrawing();
    //----------------------------------------------------------------------------------
  }

  return 0;
}
