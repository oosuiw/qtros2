#include "qtros2/main_gui.hpp"

#include <QPushButton>
#include <QLabel>
#include <QBoxLayout>
#include <QTimer>  
#include <QIcon>
#include <QScreen>
#include <QGuiApplication>

#include <std_msgs/msg/bool.hpp> 
#include <std_msgs/msg/float64.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp> 
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>  
#include <QShowEvent> // 추가: QShowEvent 선언을 포함하기 위해 헤더 추가


MainGUI::MainGUI(const std::shared_ptr<Ros2Node>& ros2_node, QWidget* parent)
  : QMainWindow(parent)
  , ros2_node(ros2_node)
  , is_on(false) // 초기 상태 OFF로 설정
  , lateral_offset(0.0) // 초기 오프셋 값 설정
{
  main_widget = new QWidget(this);
  main_widget->setStyleSheet("background-color: #1F3347;");

  QVBoxLayout* main_layout = new QVBoxLayout;

  // 라벨 추가
  QLabel* offset_label = new QLabel("Lateral Offset: 0.0", this);
  offset_label->setStyleSheet(
      "QLabel {"
      "color: white;"
      "font-weight: bold;"
      "font-size: 24px;"
      "}"
  );
  offset_label->setAlignment(Qt::AlignCenter);
  main_layout->addWidget(offset_label);
  main_layout->setSpacing(20);
  main_layout->setMargin(20);

  // 버튼 추가
  toggle_button = new QPushButton("OFF", this); // 초기 텍스트 OFF
  toggle_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  
  toggle_button->setStyleSheet(
      "QPushButton {"
      "border-radius: 50px;"
      "background-color: #CC0000;"
      "color: white;"
      "font-weight: bold;"
      "}"
  );

  connect(toggle_button, &QPushButton::clicked, this, &MainGUI::toggle_operation_mode);

  // 버튼을 main_layout에 추가
  main_layout->addWidget(toggle_button, 1); // 버튼의 크기를 절반으로 줄이기 위해 stretch factor를 1로 설정

  // 사각형 두 개 추가
  QHBoxLayout* rectangles_layout = new QHBoxLayout;

  // 왼쪽 버튼 추가
  QPushButton* left_arrow_button = new QPushButton("\u2190", this); // 왼쪽 화살표 문자 추가
  left_arrow_button->setStyleSheet(
      "QPushButton {"
      "background-color: #4A90E2;"
      "border-radius: 10px;"
      "color: white;"
      "font-weight: bold;"
      "font-size: 36px;"
      "}"
  );
  left_arrow_button->setMinimumSize(180, 180); // 버튼 크기 증가
  rectangles_layout->addWidget(left_arrow_button);
  connect(left_arrow_button, &QPushButton::clicked, this, [this, offset_label]() {
    decrease_lateral_offset();
    offset_label->setText(QString("Lateral Offset: %1").arg(lateral_offset, 0, 'f', 1));
  });

  // 오른쪽 버튼 추가
  QPushButton* right_arrow_button = new QPushButton("\u2192", this); // 오른쪽 화살표 문자 추가
  right_arrow_button->setStyleSheet(
      "QPushButton {"
      "background-color: #50E3C2;"
      "border-radius: 10px;"
      "color: white;"
      "font-weight: bold;"
      "font-size: 36px;"
      "}"
  );
  right_arrow_button->setMinimumSize(180, 180); // 버튼 크기 증가
  rectangles_layout->addWidget(right_arrow_button);
  connect(right_arrow_button, &QPushButton::clicked, this, [this, offset_label]() {
    increase_lateral_offset();
    offset_label->setText(QString("Lateral Offset: %1").arg(lateral_offset, 0, 'f', 1));
  });

  // 사각형 레이아웃을 main_layout에 추가
  main_layout->addLayout(rectangles_layout);

  main_widget->setLayout(main_layout);
  setCentralWidget(main_widget);
  adjustFontSize();
}

QSize MainGUI::sizeHint() const
{
  return QSize(300, 400); // 크기 조정을 위해 높이를 늘림
}

void MainGUI::resizeEvent(QResizeEvent* event)
{
  QMainWindow::resizeEvent(event);
  adjustFontSize();
}

void MainGUI::adjustFontSize()
{
  int buttonSize = qMin(toggle_button->width(), toggle_button->height());
  int fontSize = buttonSize / 3;

  QFont buttonFont = toggle_button->font();
  buttonFont.setPointSize(fontSize);
  toggle_button->setFont(buttonFont);
}

void MainGUI::toggle_operation_mode()
{
  if (is_on)
  {
    ros2_node->operation_mode_req_off();
    toggle_button->setText("OFF");
    toggle_button->setStyleSheet(
        "QPushButton {"
        "border-radius: 50px;"
        "background-color: #CC0000;"
        "color: white;"
        "font-weight: bold;"
        "}"
    );
    is_on = false;
  }
  else
  {
    ros2_node->operation_mode_req_on();
    toggle_button->setText("ON");
    toggle_button->setStyleSheet(
        "QPushButton {"
        "border-radius: 50px;"
        "background-color: #00CC66;"
        "color: white;"
        "font-weight: bold;"
        "}"
    );
    is_on = true;
  }
}

void MainGUI::decrease_lateral_offset()
{
  lateral_offset -= 0.1;
  if (lateral_offset < -0.5) {
    lateral_offset = -0.5;
  }
  ros2_node->publish_lateral_offset(lateral_offset); // Ros2Node 객체를 통해 퍼블리시
}

void MainGUI::increase_lateral_offset()
{
  lateral_offset += 0.1;
  if (lateral_offset > 0.5) {
    lateral_offset = 0.5;
  }
  ros2_node->publish_lateral_offset(lateral_offset); // Ros2Node 객체를 통해 퍼블리시
}

void MainGUI::publish_lateral_offset()
{
  std_msgs::msg::Float64 msg;
  msg.data = lateral_offset;
  publisher_lateral_offset_->publish(msg);
}

void MainGUI::showEvent(QShowEvent* event)
{
  QMainWindow::showEvent(event);

  // 화면 크기를 가져와서 오른쪽 끝으로 설정
  QRect screenGeometry = QGuiApplication::primaryScreen()->geometry();
  int screenWidth = screenGeometry.width();
  int screenHeight = screenGeometry.height();

  // 윈도우 위치를 오른쪽 끝으로 설정하고, 높이를 최대, 폭을 최소로 설정
  setGeometry(screenWidth - 300, 0, 300, screenHeight);
}

MainGUI::~MainGUI()
{
}

