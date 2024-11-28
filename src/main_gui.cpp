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
{
  main_widget = new QWidget(this);
  main_widget->setStyleSheet("background-color: #1F3347;");

  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->setSpacing(20);
  main_layout->setMargin(20);

  status_label = new QLabel("Control Mode : OFF", this); 
  status_label->setStyleSheet("color: white; font-size: 32px;");
  status_label->setAlignment(Qt::AlignCenter);

  QHBoxLayout* status_layout = new QHBoxLayout;
  status_layout->addWidget(status_label, 1, Qt::AlignCenter);

  main_layout->addLayout(status_layout);

  true_button = new QPushButton("ON", this);
  false_button = new QPushButton("OFF", this);

  true_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  false_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  
  true_button->setStyleSheet(
      "QPushButton {"
      "border-radius: 50px;"
      "background-color: #00CC66;"
      "color: white;"
      "font-weight: bold;"
      "}"
  );

  false_button->setStyleSheet(
      "QPushButton {"
      "border-radius: 50px;"
      "background-color: #CC0000;"
      "color: white;"
      "font-weight: bold;"
      "}"
  );

  connect(true_button, &QPushButton::clicked, this, &MainGUI::operation_mode_on);
  connect(false_button, &QPushButton::clicked, this, &MainGUI::operation_mode_off);

  main_layout->addWidget(true_button);
  main_layout->addWidget(false_button);

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
  int buttonSize = qMin(true_button->width(), true_button->height());
  int fontSize = buttonSize / 3;

  QFont font = status_label->font();
  font.setPointSize(fontSize);
  status_label->setFont(font);

  QFont buttonFont = true_button->font();
  buttonFont.setPointSize(fontSize);
  true_button->setFont(buttonFont);
  false_button->setFont(buttonFont);
}

void MainGUI::operation_mode_on() 
{
  ros2_node->operation_mode_req_on();
  status_label->setText("Control Mode : ON");  
}

void MainGUI::operation_mode_off()
{
  ros2_node->operation_mode_req_off();
  status_label->setText("Control Mode : OFF"); 
}

MainGUI::~MainGUI()
{
}
