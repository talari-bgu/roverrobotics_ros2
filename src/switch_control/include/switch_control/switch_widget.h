#ifndef SWITCH_WIDGET_H
#define SWITCH_WIDGET_H

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


namespace switch_panel
{
    class SwitchWidget : public QWidget
    {
        Q_OBJECT
    public:
        explicit SwitchWidget(QWidget *parent = nullptr);
        ~SwitchWidget() override;
    
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

#endif // SWITCH_WIDGET_H