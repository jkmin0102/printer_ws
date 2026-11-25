// Presentation only: use Gazebo's native pane API in this client process.
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/gui/MainWindow.hh>
#include <QTimer>

namespace namo_rviz {
class GazeboView : public gazebo::GUIPlugin {
public:
  void Load(sdf::ElementPtr) override {
    setFixedSize(0, 0);
    auto timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this, timer]() {
      auto window = gazebo::gui::get_main_window();
      if (!window || !window->isVisible()) return;
      window->SetLeftPaneVisibility(false);
      hide();
      timer->stop();  // The user can reopen the pane afterwards.
    });
    timer->start(250);
  }
};
GZ_REGISTER_GUI_PLUGIN(GazeboView)
}
