
#include "Crane3R.hpp"

#include "Angle.hpp"
#include "kine/Kine.hpp"
#include "threepp/threepp.hpp"

#include <future>

#include "kine/ik/CCDSolver.hpp"
#include "kine/ik/DLSSolver.hpp"
#include "kine/ik/DNNSolver.hpp"
#include "threepp/extras/imgui/ImguiContext.hpp"

using namespace threepp;


struct MyUI: ImguiContext {

    bool jointMode = true;
    bool posMode = false;
    bool enableController = false;

    Vector3 pos;
    std::vector<float> values;

    explicit MyUI(const Canvas& canvas, const kine::Kine& kine)
        : ImguiContext(canvas.windowPtr()),
          limits(kine.limits()),
          values(kine.meanAngles()),
          solverNames_{"CCD", "DLS"} {

        solvers_.emplace_back(std::make_unique<kine::CCDSolver>());
        solvers_.emplace_back(std::make_unique<kine::DLSSolver>());
#ifdef KINE_WITH_DNN
        solvers_.emplace_back(std::make_unique<kine::DNNSolver>("data/Crane3R/crane3r.onnx"));
        solverNames_.emplace_back("DNN");
#endif

        pos.setFromMatrixPosition(kine.calculateEndEffectorTransformation(values).elements);
    }

    void onRender() override {

        ImGui::SetNextWindowPos({}, 0, {});
        ImGui::SetNextWindowSize({}, 0);
        ImGui::Begin("Crane3R");

        if (ImGui::BeginCombo("IK Solver", solverNames_[current_solver_])) {
            for (int i = 0; i < solverNames_.size(); i++) {
                const bool isSelected = (current_solver_ == i);
                if (ImGui::Selectable(solverNames_[i], isSelected)) {
                    current_solver_ = i;
                }
            }
            ImGui::EndCombo();
        }

        ImGui::Text("Target angles");
        ImGui::SliderFloat("j1", &values[0], *limits[0].min(), *limits[0].max());
        jointMode = jointMode || ImGui::IsItemEdited();
        ImGui::SliderFloat("j2", &values[1], *limits[1].min(), *limits[1].max());
        jointMode = jointMode || ImGui::IsItemEdited();
        ImGui::SliderFloat("j3", &values[2], *limits[2].min(), *limits[2].max());
        jointMode = jointMode || ImGui::IsItemEdited();

        posMode = !jointMode;

        ImGui::Text("Target pos");
        ImGui::SliderFloat("px", &pos.x, -10, 10);
        posMode = posMode || ImGui::IsItemEdited();
        ImGui::SliderFloat("py", &pos.y, 0, 10);
        posMode = posMode || ImGui::IsItemEdited();
        ImGui::SliderFloat("pz", &pos.z, 0, 10);
        posMode = posMode || ImGui::IsItemEdited();

        jointMode = !posMode;

        ImGui::Checkbox("controller", &enableController);

        ImGui::End();
    }

    [[nodiscard]] kine::IKSolver& getSelectedSolver() const {

        return *solvers_.at(current_solver_);
    }

private:
    std::vector<kine::KineLimit> limits;

    int current_solver_ = 0;
    std::vector<const char*> solverNames_;

    std::vector<std::unique_ptr<kine::IKSolver>> solvers_;
};

auto createGrid() {

    unsigned int size = 30;
    auto material = ShadowMaterial::create();
    auto plane = Mesh::create(PlaneGeometry::create(size, size), material);
    plane->rotation.x = -math::PI / 2;
    plane->receiveShadow = true;

    auto grid = GridHelper::create(size, size, Color::yellowgreen);
    grid->rotation.x = math::PI / 2;
    plane->add(grid);

    return plane;
}

int main() {

    Canvas canvas{"Crane3R", {{"size", WindowSize{1280, 720}}, {"antialiasing", 8}}};
    GLRenderer renderer{canvas.size()};
    renderer.autoClear = false;
    renderer.shadowMap().enabled = true;
    renderer.setClearColor(Color::aliceblue);

    auto camera = PerspectiveCamera::create(60, canvas.aspect(), 0.01, 100);
    camera->position.set(-15, 8, 15);

    OrbitControls controls(*camera, canvas);

    auto scene = Scene::create();

    auto grid = createGrid();
    scene->add(grid);

    auto endEffectorHelper = AxesHelper::create(1);
    endEffectorHelper->visible = false;
    scene->add(endEffectorHelper);

    auto light1 = AmbientLight::create(Color::white);
    auto light2 = DirectionalLight::create(Color::white);
    light2->shadow->camera->as<OrthographicCamera>()->top = 15;
    light2->shadow->camera->as<OrthographicCamera>()->bottom = -15;
    light2->shadow->camera->as<OrthographicCamera>()->left = 15;
    light2->shadow->camera->as<OrthographicCamera>()->right = -15;
    light2->position.set(-100, 100, 50);
    light2->castShadow = true;
    scene->add(light1);
    scene->add(light2);

    auto targetHelper = AxesHelper::create(2);
    targetHelper->visible = false;
    scene->add(targetHelper);

    HUD hud(canvas.size());
    FontLoader fontLoader;
    const auto font = fontLoader.defaultFont();

    TextGeometry::Options opts(font, 40, 2);
    auto handle = Text2D(opts, "Loading Crane3R..");
    handle.setColor(Color::black);
    hud.add(handle, HUD::Options()
                            .setNormalizedPosition({0.5, 0.5})
                            .setHorizontalAlignment(HUD::HorizontalAlignment::CENTER)
                            .setVerticalAlignment(HUD::VerticalAlignment::CENTER));

    kine::Kine kine = kine::KineBuilder()
                              .addRevoluteJoint(Vector3::Y(), {-90.f, 90.f})
                              .addLink(Vector3::Y() * 4.2)
                              .addRevoluteJoint(Vector3::X(), {-80.f, 0.f})
                              .addLink(Vector3::Z() * 7)
                              .addRevoluteJoint(Vector3::X(), {40.f, 140.f})
                              .addLink(Vector3::Z() * 5.2)
                              .build();

    TaskManager tm;
    std::shared_ptr<Crane3R> crane;
    auto future = std::async([&] {
        crane = Crane3R::create();
        crane->setTargetValues(asAngles(kine.meanAngles(), kine::Angle::Repr::DEG));
        crane->traverseType<Mesh>([](Mesh& m) {
            m.castShadow = true;
        });

        tm.invokeLater([&, crane] {
            hud.remove(handle);
            scene->add(crane);
            endEffectorHelper->visible = true;
        });
    });


    canvas.onWindowResize([&](WindowSize size) {
        camera->aspect = size.aspect();
        camera->updateProjectionMatrix();
        renderer.setSize(size);

        hud.setSize(size);
    });

    IOCapture capture{};
    capture.preventMouseEvent = [] {
        return ImGui::GetIO().WantCaptureMouse;
    };
    canvas.setIOCapture(&capture);

    Clock clock;
    MyUI ui(canvas, kine);
    canvas.animate([&]() {
        const auto dt = clock.getDelta();

        renderer.clear();
        renderer.render(*scene, *camera);

        tm.handleTasks();

        if (crane) {

            ui.render();

            const auto endEffectorPosition = kine.calculateEndEffectorTransformation(inDegrees(crane->getValues()));
            endEffectorHelper->position.setFromMatrixPosition(endEffectorPosition.elements);

            targetHelper->position.copy(ui.pos);

            if (ui.jointMode) {
                ui.pos.setFromMatrixPosition(kine.calculateEndEffectorTransformation(ui.values).elements);
                targetHelper->visible = false;
            }
            if (ui.posMode) {
                ui.values = ui.getSelectedSolver().solveIK(kine, ui.pos, inDegrees(crane->getValues()));
                targetHelper->visible = true;
            }

            crane->controllerEnabled = ui.enableController;
            crane->setTargetValues(asAngles(ui.values, kine::Angle::Repr::DEG));

            crane->update(dt);

        } else {

            hud.apply(renderer);
        }
    });

    future.get();
}
