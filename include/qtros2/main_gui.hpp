#ifndef MAIN_GUI_HPP
#define MAIN_GUI_HPP

#include <QMainWindow>
#include <QLabel> 
#include <QPushButton>
#include <memory>
#include "qtros2/ros2node.hpp"

class MainGUI : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainGUI(const std::shared_ptr<Ros2Node>& ros2_node, QWidget* parent = nullptr);
  ~MainGUI() override;

protected:
  void resizeEvent(QResizeEvent* event) override;
  QSize sizeHint() const override; 

private:
  void adjustFontSize();  
  void toggle_operation_mode();

  std::shared_ptr<Ros2Node> ros2_node;
  QLabel* status_label;  
  QWidget* main_widget;
  QPushButton* toggle_button;
  bool is_on; // 상태를 나타내는 변수
};

#endif // MAIN_GUI_HPP
