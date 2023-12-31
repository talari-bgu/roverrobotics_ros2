#ifndef DEMO_WIDGET_H
#define DEMO_WIDGET_H

#include <QWidget>
#include <QSlider>
#include <QLabel>
#include <QCheckBox>

#include <memory>
#include <vector>

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#endif


namespace demo_panel
{
    class DemoWidget : public QWidget
    {
        Q_OBJECT
    public:
        explicit DemoWidget(QWidget *parent = nullptr);
        ~DemoWidget() override;
    
    private slots:
        void onSliderValueChanged(int value);
        void onCheckboxStateChanged(int state);
    
    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
        
        QSlider* modeSlider;
        QLabel* sliderLabel1;
        QLabel* sliderLabel2;
        QLabel* sliderLabel3;

        QCheckBox* checkBox;

        QLabel* infoLabel;
    };
}

#endif // DEMO_WIDGET_H