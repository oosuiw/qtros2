#include "qtros2/main_gui.hpp"

#include <QPushButton>
#include <QLabel>
#include <QBoxLayout>
#include <QTimer>  

#include <std_msgs/msg/bool.hpp> 
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp> 
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>  

MainGUI::MainGUI(const std::shared_ptr<Ros2Node>& ros2_node, QWidget* parent)
  : QMainWindow(parent)
  , ros2_node(ros2_node)
  , is_on(false) // 초기 상태 OFF로 설정
{
  main_widget = new QWidget(this);
  main_widget->setStyleSheet("background-color: #1F3347;");

  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->setSpacing(20);
  main_layout->setMargin(20);

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

  main_layout->addWidget(toggle_button);

  main_widget->setLayout(main_layout);
  setCentralWidget(main_widget);

  adjustFontSize();
}

QSize MainGUI::sizeHint() const
{
  return QSize(300, 200);
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

MainGUI::~MainGUI()
{
}
