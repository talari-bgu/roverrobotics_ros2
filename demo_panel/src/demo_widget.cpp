#include <demo_panel/demo_widget.h>
#include <QLabel>
#include <QGridLayout>
#include <QVBoxLayout>

namespace demo_panel
{
    DemoWidget::DemoWidget(QWidget *parent): QWidget(parent)
    {
        // Initialize ROS node and publisher
        node_ = std::make_shared<rclcpp::Node>("demo_widget_node");
        publisher_ = node_->create_publisher<std_msgs::msg::Int32>("loa_mode", 10);


        // Create and configure the slider
        modeSlider = new QSlider(Qt::Vertical, this);
        modeSlider->setRange(0,2);
        modeSlider->setSingleStep(1);

        QLabel *label1 = new QLabel("Manual", this);
        QLabel *label2 = new QLabel("Assisted Teleoperation", this);
        QLabel *label3 = new QLabel("Autonomous Navigation", this);

        QGridLayout *layout = new QGridLayout;



        layout->addWidget(modeSlider, 0, 0, 3, 1);
        // layout->setAlignment(modeSlider, Qt::AlignVCenter);
        
        layout->addWidget(label1, 2, 1, 1, 1);
        layout->setAlignment(label1, Qt::AlignBottom);
        layout->addWidget(label2, 1, 1, 1, 1);
        layout->setAlignment(label2, Qt::AlignVCenter);
        layout->addWidget(label3, 0, 1, 1, 1);
        layout->setAlignment(label3, Qt::AlignTop);
        
        setLayout(layout);

        // Connect the slider's valueChanged signal to our slot
        connect(modeSlider, &QSlider::valueChanged, this, &DemoWidget::onSliderValueChanged);

        // Set the initial mode text
        onSliderValueChanged(modeSlider->value());
    }

    DemoWidget::~DemoWidget()
    {
        delete modeSlider;
    }

    void DemoWidget::onSliderValueChanged(int value) {

        // std_msgs::msg::string msg;
        // switch (value) {
        //     case 0:
        //         msg.data = "manual";
        //         break;
        //     case 1:
        //         modeLabel->setText("Assisted Teleoperation");
        //         break;
        //     case 2:
        //         modeLabel->setText("Autonomous Navigation");
        //         break;
        //     default:
        //         modeLabel->setText("Unknown Mode");
        //         break;
        // }
            // Publish the current value of the slider
        std_msgs::msg::Int32 msg;
        msg.data = value;
        publisher_->publish(msg);
}
}
