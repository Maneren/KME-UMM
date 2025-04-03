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
#include <format>
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
      FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT //| FLAG_VSYNC_HINT
  );

  rlEnableBackfaceCulling();

  window.SetTargetFPS(30);

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

  const auto cube_mesh = raylib::MeshUnmanaged::Cube(2.0f, 2.0f, 2.0f);

  std::shared_ptr<Body> cube = std::make_shared<Polyhedron>(
      cube_mesh,
      100.f,
      raylib::Color::Black(),
      steel,
      raylib::Vector3{0.0f, 1.0f, 8.0f}
  );

  std::vector<std::shared_ptr<Object>> game_objects;

  game_objects.push_back(cube);

  std::shared_ptr<Body> cube1 = std::make_shared<Cuboid>(
      raylib::Vector3{2.0f, 2.0f, 2.0f},
      raylib::Color::Red(),
      rubber,
      raylib::Vector3{5.0f, 1.0f, 0.0f}
  );

  cube1->mass(1000.f);

  game_objects.push_back(cube1);

  std::shared_ptr<Body> cube2 = std::make_shared<Cuboid>(
      raylib::Vector3{2.0f, 2.f, 6.0f},
      raylib::Color::Blue(),
      lubricated,
      raylib::Vector3{-5.0f, 1.0f, 0.0f}
  );

  cube2->mass(1000.f);

  game_objects.push_back(cube2);

  std::shared_ptr<Body> cube3 = std::make_shared<Cuboid>(
      raylib::Vector3{2.0f, 2.f, 2.0f},
      raylib::Color::Yellow(),
      steel,
      raylib::Vector3{5.0f, 1.0f, -9.0f}
  );

  game_objects.push_back(cube3);

  cube3->mass(1000.f);

  std::shared_ptr<Joint> joint = std::make_shared<SpringJoint>(
      std::static_pointer_cast<Body>(cube1),
      std::static_pointer_cast<Body>(cube2),
      raylib::Color::Green(),
      1000.f,
      8.f,
      0.4f
  );

  joint->anchor_a(raylib::Vector3{-1.0f, .0f, 1.0f})
      .anchor_b(raylib::Vector3{1.0f, .0f, 1.0f});

  std::shared_ptr<Joint> joint2 = std::make_shared<SpringJoint>(
      std::static_pointer_cast<Body>(cube1),
      std::static_pointer_cast<Body>(cube3),
      raylib::Color::Green(),
      1000.f,
      8.f,
      0.4f
  );

  joint2->anchor_a(raylib::Vector3{1.0f, .0f, -1.0f})
      .anchor_b(raylib::Vector3{1.0f, .0f, 1.0f});

  std::shared_ptr<Joint> joint3 = std::make_shared<SpringJoint>(
      std::static_pointer_cast<Body>(cube1),
      std::static_pointer_cast<Body>(cube),
      raylib::Color::Green(),
      1000.f,
      8.f,
      0.4f
  );

  joint3->anchor_a(raylib::Vector3{1.0f, .0f, 1.0f})
      .anchor_b(raylib::Vector3{-1.0f, .0f, -1.0f});

  game_objects.push_back(joint);
  game_objects.push_back(joint2);
  game_objects.push_back(joint3);

  for (auto &object : game_objects) {
    object->initialize();
  }
  //--------------------------------------------------------------------------------------

  // Main game loop
  while (!window.ShouldClose()) // Detect window close button or ESC key
  {
    // Update
    //----------------------------------------------------------------------------------
    float delta = window.GetFrameTime() / 10.f;
    std::println("\n\ndelta: {}", delta);

    if (delta >= EPSILON) {

      // Apply gravity
      cube1->apply_force(GRAVITY * cube1->mass());
      cube2->apply_force(GRAVITY * cube2->mass());
      cube3->apply_force(GRAVITY * cube3->mass());

      camera.Update(CAMERA_THIRD_PERSON);

      // Update all alive objects
      for (const auto &object : game_objects) {
        if (object->alive) {
          object->update(delta);
        }
      }

      // Remove dead objects
      std::erase_if(game_objects, [](const auto &object) {
        return !object->alive;
      });

      const auto move_force = 10000.f;

      if (IsKeyDown(KEY_SPACE)) {
        cube1->apply_force(-GRAVITY * 1500.f);
      }

      if (IsKeyDown(KEY_W)) {
        cube1->apply_force(raylib::Vector3{0.0f, 0.0f, -move_force});
      }

      if (IsKeyDown(KEY_S)) {
        cube1->apply_force(raylib::Vector3{0.0f, 0.0f, move_force});
      }

      if (IsKeyDown(KEY_A)) {
        cube1->apply_force(raylib::Vector3{-move_force, 0.0f, 0.0f});
      }

      if (IsKeyDown(KEY_D)) {
        cube1->apply_force(raylib::Vector3{move_force, 0.0f, 0.0f});
      }

      camera.target = (cube1->position() + cube2->position()) / 2.f;
    }
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

    const auto text = std::format("Velocity: ({})", cube1->angular_velocity());
    DrawText(&text.front(), 10, 30, 20, raylib::Color::Black());
    const auto text2 =
        std::format("Acceleration: ({})", cube1->angular_acceleration());
    DrawText(&text2.front(), 10, 50, 20, raylib::Color::Black());

    window.EndDrawing();
    //----------------------------------------------------------------------------------
  }

  return 0;
}
