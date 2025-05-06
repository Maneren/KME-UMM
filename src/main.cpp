#include "body.hpp"
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

  // Cuboid test
  // std::shared_ptr<Body> cube = std::make_shared<Cuboid>(
  //     raylib::Vector3{0.4f, 1.0f, 0.1f},
  //     raylib::Color::Red(),
  //     rubber,
  //     raylib::Vector3{0.0f, 8.0f, 0.0f}
  // );
  // cube->mass(1.f);

  // Angular momentum test
  // std::shared_ptr<Body> cube = std::make_shared<Polyhedron>(
  //     raylib::Mesh::Cube(0.5f, 1.0f, 0.05f),
  //     7500.f,
  //     raylib::Color::Red(),
  //     rubber,
  //     raylib::Vector3{0.0f, 8.0f, 0.0f}
  // );
  //
  // cube->initialize();
  //
  // game_objects.push_back(cube);
  //
  // cube->apply_force({0.0f, 0.0f, 100.f}, raylib::Vector3{0.00001f, 3.0f,
  // 0.0f});

  // 4-body test
  auto cube1 = std::make_shared<Polyhedron>(
      raylib::Mesh::Cube(2.0f, 2.0f, 2.0f),
      3000.f,
      raylib::Color::Red(),
      rubber,
      raylib::Vector3{5.0f, 1.0f, 0.0f}
  );

  game_objects.push_back(cube1);

  auto cube2 = std::make_shared<Polyhedron>(
      raylib::Mesh::Cube(2.0f, 2.f, 6.0f),
      3000.f,
      raylib::Color::Blue(),
      lubricated,
      raylib::Vector3{-5.0f, 1.0f, 0.0f}
  );

  game_objects.push_back(cube2);

  auto cube3 = std::make_shared<Polyhedron>(
      raylib::Mesh::Cube(2.0f, 2.0f, 2.0f),
      3000.f,
      raylib::Color::Yellow(),
      steel,
      raylib::Vector3{5.0f, 1.0f, -9.0f}
  );

  game_objects.push_back(cube3);

  auto cube4 = std::make_shared<Polyhedron>(
      raylib::Mesh::Cube(2.0f, 2.0f, 2.0f),
      3000.f,
      raylib::Color::Gray(),
      steel,
      raylib::Vector3{0.0f, 1.0f, 8.0f}
  );

  game_objects.push_back(cube4);

  std::shared_ptr<Joint> joint = std::make_shared<SpringJoint>(
      std::static_pointer_cast<Body>(cube1),
      std::static_pointer_cast<Body>(cube2),
      raylib::Color::Green(),
      1000.f,
      8.f,
      2.0f
  );

  joint->anchor_a(raylib::Vector3{-1.0f, .0f, 1.0f})
      .anchor_b(raylib::Vector3{1.0f, .0f, 1.0f});

  auto joint2 = std::make_shared<SpringJoint>(
      std::static_pointer_cast<Body>(cube1),
      std::static_pointer_cast<Body>(cube3),
      raylib::Color::Green(),
      1000.f,
      8.f,
      2.0f
  );

  joint2->anchor_a(raylib::Vector3{1.0f, .0f, -1.0f})
      .anchor_b(raylib::Vector3{1.0f, .0f, 1.0f});

  auto joint3 = std::make_shared<SpringJoint>(
      std::static_pointer_cast<Body>(cube1),
      std::static_pointer_cast<Body>(cube4),
      raylib::Color::Green(),
      1000.f,
      8.f,
      2.0f
  );

  joint3->anchor_a(raylib::Vector3{1.0f, .0f, 1.0f})
      .anchor_b(raylib::Vector3{-1.0f, .0f, -1.0f});

  game_objects.push_back(joint);
  game_objects.push_back(joint2);
  game_objects.push_back(joint3);

  // // Collisions test
  // std::shared_ptr<Polyhedron> cube1 = std::make_shared<Polyhedron>(
  //     raylib::Mesh::Cube(2.0f, 2.0f, 2.0f),
  //     1000.f,
  //     raylib::Color::Red(),
  //     steel,
  //     raylib::Vector3{0.0f, 1.0f, 0.0f}
  // );
  //
  // game_objects.push_back(cube1);
  //
  // std::shared_ptr<Polyhedron> cube2 = std::make_shared<Polyhedron>(
  //     raylib::Mesh::Cube(2.0f, 2.0f, 4.0f),
  //     1000.f,
  //     raylib::Color::Blue(),
  //     steel,
  //     raylib::Vector3{6.0f, 1.0f, 0.0f},
  //     raylib::Quaternion::FromAxisAngle(
  //         raylib::Vector3{0.0f, 1.0f, 0.0f}, PI / 6
  //     ) *
  //         raylib::Quaternion::FromAxisAngle(
  //             raylib::Vector3{0.0f, 0.0f, 1.0f}, PI / 6
  //         )
  // );

  game_objects.push_back(cube2);

  for (auto &object : game_objects) {
    object->initialize();
  }
  //--------------------------------------------------------------------------------------

  // Physics step time in seconds
  constexpr float STEP_TIME = 1.e-3f;
  double physics_time = 0.0f;

  cube1->apply_force(raylib::Vector3{8000.0f / STEP_TIME, 0.0f, 0.0f});

  // Main game loop
  while (!window.ShouldClose()) // Detect window close button or ESC key
  {
    // Update
    //----------------------------------------------------------------------------------
    const auto simulation_time = window.GetTime();

    float total_energy;

    while (physics_time < simulation_time) {
      // std::println(
      //     "\n\nStepping physics... {} -> {}",
      //     physics_time,
      //     physics_time + STEP_TIME
      // );
      physics_time += static_cast<double>(STEP_TIME);

      // Apply gravity
      // cube1->apply_force(GRAVITY * cube1->mass());
      // cube2->apply_force(GRAVITY * cube2->mass());
      // cube3->apply_force(GRAVITY * cube3->mass());

      total_energy = 0.0f;

      // Update all alive objects
      for (const auto &object : game_objects) {
        if (!object->alive)
          continue;

        if (const auto body = std::dynamic_pointer_cast<Body>(object); body) {
          total_energy += body->linear_energy() + body->angular_energy();
          // body->apply_force(GRAVITY * body->mass());
          // const auto state = body->state();
          //
          // ode<Body::State>(Body::State::update, state, STEP_TIME);
        } else if (const auto spring =
                       std::dynamic_pointer_cast<SpringJoint>(object);
                   spring) {
          total_energy += spring->potential_energy();
        }

        object->update(STEP_TIME);
      }

      const auto collision = detect_polyhedron_collision(cube1, cube2);

      if (collision.has_value()) {
        const auto &value = collision.value();
        value.resolve();
      }

      // Remove dead objects
      std::erase_if(game_objects, [](const auto &object) {
        return !object->alive;
      });
    }

    const auto move_force = 0.2f / STEP_TIME * cube1->mass();

    if (IsKeyDown(KEY_SPACE)) {
      cube1->apply_force(raylib::Vector3{0.0f, move_force, 0.0f});
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
    // camera.target = cube->position();
    // camera.target = cube1->position();
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

    DrawText(
        std::format("Total energy: {:.2g} J", total_energy).c_str(),
        10,
        30,
        20,
        raylib::Color::Lime()
    );

    window.EndDrawing();
    //----------------------------------------------------------------------------------
  }

  return 0;
}
