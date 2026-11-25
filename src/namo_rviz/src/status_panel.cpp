#include "namo_rviz/status_panel.hpp"
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <QVBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMetaObject>
#include <QScrollArea>
#include <QPushButton>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/render_panel.hpp>
#include <algorithm>
#include <pluginlib/class_list_macros.hpp>
namespace namo_rviz {
StatusPanel::StatusPanel(QWidget *parent):Panel(parent) {
  auto layout=new QVBoxLayout(this);
  label_=new QLabel("미션 상태 수신 대기",this);
  label_->setWordWrap(true);label_->setAlignment(Qt::AlignTop);
  label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  label_->setMinimumWidth(260);label_->setTextFormat(Qt::PlainText);
  freshness_=new QLabel("",this);
  auto scroll=new QScrollArea(this);
  scroll->setWidgetResizable(true);scroll->setFrameShape(QFrame::NoFrame);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  scroll->setWidget(label_);scroll->setMinimumHeight(120);
  auto fit = new QPushButton("지도 전체 맞춤", this);
  connect(fit, &QPushButton::clicked, this, [this](){fitted_size_=QSize();fitMap();});
  layout->addWidget(scroll);layout->addWidget(fit);layout->addWidget(freshness_);
  auto timer=new QTimer(this);
  connect(timer,&QTimer::timeout,this,[this](){
    const auto age=std::chrono::duration<double>(std::chrono::steady_clock::now()-last_).count();
    freshness_->setText(age>2.?"상태 수신 없음 / 종료 또는 연결 확인":"상태 갱신 중");
    fitMap();
  });timer->start(500);
}
void StatusPanel::onInitialize() {
  auto node=getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  map_sub_=node->create_subscription<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).transient_local(),
    [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg){
      if(msg->header.frame_id!="map")return;
      const auto info=msg->info;
      QMetaObject::invokeMethod(this,[this,info](){
        map_width_=info.width*info.resolution;map_height_=info.height*info.resolution;
        center_x_=info.origin.position.x+map_width_/2.;
        center_y_=info.origin.position.y+map_height_/2.;
        fitted_size_=QSize();fitMap();
      },Qt::QueuedConnection);
    });
  sub_=node->create_subscription<std_msgs::msg::String>("/namo/status",rclcpp::QoS(1).transient_local(),
    [this](std_msgs::msg::String::ConstSharedPtr msg){
      const auto data=QString::fromStdString(msg->data);
      QMetaObject::invokeMethod(this,[this,data](){
        const auto j=QJsonDocument::fromJson(data.toUtf8()).object();
        if(j.isEmpty()){label_->setText("상태 메시지 해석 실패");return;}
        last_=std::chrono::steady_clock::now();
        auto seconds=[&j](const char *key){return j[key].isDouble()?QString::number(j[key].toDouble(),'f',1)+"초":QString(j["choice"].toString().isEmpty()?"미계산":"후보 없음");};
        auto choice=j["choice"].toString();
        if(choice=="REMOVE")choice="치우고 직진";
        else if(choice=="DETOUR")choice="우회";
        else if(choice=="CLEAR")choice="장애물 없이 주행";
        else choice="판단 전";
        QString text="NAMO 진행 상태\n\n현재: "+j["label"].toString()+"\n";
        text+="선택: "+choice+"\n";
        text+="베이스: "+QString(j["drive_enabled"].toBool()?"주행 허용":"주행 잠금")+"\n";
        text+=j["drive_reason"].toString()+"\n\n";
        text+="판단 당시 예상 총시간\n  치우고 직진: "+seconds("T_remove")+"\n  우회: "+seconds("T_detour")+"\n";
        text+="이유: "+j["reason"].toString()+"\n\n";
        text+="팔 상태: "+j["arm_state_label"].toString()+" / "+j["arm_stage_label"].toString()+"\n";
        text+="실측 경과 (시뮬레이션): "+seconds("elapsed")+"\n\n";
        text+="인식 모드: "+j["perception_mode"].toString()+"\n";
        if(j["state"].toString()=="RECOGNIZE" || j["state"].toString()=="FAILED")
          text+="카메라: "+j["camera_reason"].toString()+"\n";
        text+="보라 = 현재 주행 경로 (계속 갱신)\n파랑 = 제거 후 후보\n주황 = 장애물 유지 우회\n초록 점 = LiDAR 관측\n빨강 점 = 감지 위치 기록 / 노랑 점 = 판단 위치\n\n";
        text+=j["execution_mode"].toString();
        if(!j["navigation_notice"].toString().isEmpty())text+="\n\n주행 진단: "+j["navigation_notice"].toString();
        if(!j["failure"].toString().isEmpty())text+="\n\n중단 사유: "+j["failure_label"].toString();
        label_->setText(text);
      },Qt::QueuedConnection);
    });
}
void StatusPanel::fitMap() {
  if(map_width_<=0. || map_height_<=0. || !getDisplayContext())return;
  auto manager=getDisplayContext()->getViewManager();
  auto view=manager->getCurrent();auto panel=manager->getRenderPanel();
  if(!view || !panel || view->getClassId()!="rviz_default_plugins/TopDownOrtho")return;
  const auto size=panel->size();
  if(size.width()<100 || size.height()<100 || fitted_size_==size)return;
  view->subProp("Scale")->setValue(std::min((size.width()-24.)/map_width_,(size.height()-24.)/map_height_));
  view->subProp("X")->setValue(center_x_);view->subProp("Y")->setValue(center_y_);
  fitted_size_=size; // Manual zoom/pan is preserved until a resize or button click.
  getDisplayContext()->queueRender();
}
}
PLUGINLIB_EXPORT_CLASS(namo_rviz::StatusPanel,rviz_common::Panel)
