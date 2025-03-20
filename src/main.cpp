#include "consts.hpp"
// #include "joints/force.hpp"
#include "cuboid.hpp"
#include "joints/force.hpp"
#include "joints/spring.hpp"
#include "material.hpp"
#include "object.hpp"
#include <Camera3D.hpp>
#include <Color.hpp>
#include <Vector3.hpp>
#include <Window.hpp>
#include <memory>
#include <raylib.h>
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

  std::shared_ptr<Body> cube1 = std::make_shared<Cuboid>(
      raylib::Vector3{2.0f, 2.0f, 2.0f},
      raylib::Color::Red(),
      rubber,
      raylib::Vector3{5.0f, 0.0f, 0.0f}
  );

  cube1->mass(1.f);

  game_objects.push_back(cube1);

  std::shared_ptr<Body> cube2 = std::make_shared<Cuboid>(
      raylib::Vector3{2.0f, 2.f, 6.0f},
      raylib::Color::Blue(),
      lubricated,
      raylib::Vector3{-5.0f, 0.0f, 0.0f}
  );

  cube2->mass(1.f);

  game_objects.push_back(cube2);

  std::shared_ptr<Joint> joint = std::make_shared<SpringJoint>(
      std::static_pointer_cast<Body>(cube1),
      std::static_pointer_cast<Body>(cube2),
      raylib::Color::Green(),
      1.f,
      2.f,
      0.9f
  );

  joint->anchor_a(raylib::Vector3{-1.0f, .0f, 1.0f})
      .anchor_b(raylib::Vector3{1.0f, .0f, 1.0f});

  game_objects.push_back(joint);

  for (auto &object : game_objects) {
    object->initialize();
  }

  raylib::Vector3 random_force{0.0f, 0.0f, 0.0f};

  //--------------------------------------------------------------------------------------

  // Main game loop
  while (!window.ShouldClose()) // Detect window close button or ESC key
  {
    // Update
    //----------------------------------------------------------------------------------
    const float delta = window.GetFrameTime();

    // Apply gravity
    cube1->apply_force(GRAVITY);
    cube2->apply_force(GRAVITY);

    camera.Update(CAMERA_THIRD_PERSON);

    for (const auto &object : game_objects) {
      object->update(delta);
    }

    if (IsKeyDown(KEY_SPACE)) {
      cube1->apply_force(-GRAVITY * 1.5f);
    }

    if (IsKeyDown(KEY_W)) {
      cube1->apply_force(raylib::Vector3{0.0f, 0.0f, -10.0f});
    }

    if (IsKeyDown(KEY_S)) {
      cube1->apply_force(raylib::Vector3{0.0f, 0.0f, 10.0f});
    }

    if (IsKeyDown(KEY_A)) {
      cube1->apply_force(raylib::Vector3{-10.0f, 0.0f, 0.0f});
    }

    if (IsKeyDown(KEY_D)) {
      cube1->apply_force(raylib::Vector3{10.0f, 0.0f, 0.0f});
    }

    camera.target = (cube1->position() + cube2->position()) / 2.f;
    //----------------------------------------------------------------------------------

    // Draw
    //----------------------------------------------------------------------------------
    window.BeginDrawing();

    window.ClearBackground(raylib::Color::RayWhite());

    camera.BeginMode();

    DrawGrid(50, 1.0f);

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
