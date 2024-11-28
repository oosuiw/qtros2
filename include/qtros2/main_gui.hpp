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
  void operation_mode_on();
  void operation_mode_off();


  std::shared_ptr<Ros2Node> ros2_node;
  QLabel* status_label;  
  QWidget* main_widget;
  QPushButton* true_button;
  QPushButton* false_button;
};

#endif // MAIN_GUI_HPP
