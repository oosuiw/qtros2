#ifndef MAIN_GUI_HPP
#define MAIN_GUI_HPP

#include <QMainWindow>
#include <QLabel> 
#include <QPushButton>
#include <memory>
#include "qtros2/ros2node.hpp"
#include <QShowEvent> // 추가: QShowEvent 선언을 포함하기 위해 헤더 추가


class MainGUI : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainGUI(const std::shared_ptr<Ros2Node>& ros2_node, QWidget* parent = nullptr);
  void showEvent(QShowEvent* event) override;

  ~MainGUI() override;

protected:
  void resizeEvent(QResizeEvent* event) override;
  QSize sizeHint() const override;

private slots:
  void decrease_lateral_offset();
  void increase_lateral_offset();
  void toggle_operation_mode();
  void publish_lateral_offset();

private:
  std::shared_ptr<Ros2Node> ros2_node;
  QLabel* status_label;
  QWidget* main_widget;
  QPushButton* toggle_button;
  bool is_on;
  double lateral_offset;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_lateral_offset_;

  void adjustFontSize();
};


#endif // MAIN_GUI_HPP
