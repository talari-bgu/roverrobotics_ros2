#include <switch_control/switch_widget.h>
#include <QLabel>
#include <QGridLayout>
#include <QVBoxLayout>

namespace switch_panel
{
    SwitchWidget::SwitchWidget(QWidget *parent): QWidget(parent)
    {
        // Initialize ROS node and publisher
        node_ = std::make_shared<rclcpp::Node>("switch_widget_node");
        publisher_ = node_->create_publisher<std_msgs::msg::Int32>("loa_mode", 10);



        // Create elements
        modeSlider = new QSlider(Qt::Vertical, this);
        modeSlider->setRange(0,2);
        modeSlider->setSingleStep(1);

        sliderLabel1 = new QLabel("Manual", this);
        sliderLabel2 = new QLabel("Assisted Teleoperation", this);
        sliderLabel3 = new QLabel("Autonomous Navigation", this);

        checkBox = new QCheckBox("RI", this);

        infoLabel = new QLabel("HI is ON ", this);
        infoLabel->setStyleSheet("QLabel { color : black; font-weight: bold;}"); // Set text color to red

        QGridLayout *layout = new QGridLayout;

        layout->addWidget(infoLabel, 0, 0, 1, 3);
        layout->setAlignment(infoLabel, Qt::AlignHCenter);
        layout->addWidget(modeSlider, 1, 0, 3, 1);
        layout->addWidget(checkBox, 1, 2, 1, 1); 
        layout->setAlignment(checkBox, Qt::AlignVCenter);
        
        layout->addWidget(sliderLabel1, 3, 1, 1, 1);
        layout->setAlignment(sliderLabel1, Qt::AlignBottom);
        layout->addWidget(sliderLabel2, 2, 1, 1, 1);
        layout->setAlignment(sliderLabel2, Qt::AlignVCenter);
        layout->addWidget(sliderLabel3, 1, 1, 1, 1);
        layout->setAlignment(sliderLabel3, Qt::AlignTop);
        
        setLayout(layout);

        // Connect the slider's valueChanged signal to our slot
        connect(modeSlider, &QSlider::valueChanged, this, &SwitchWidget::onSliderValueChanged);

        // Connect the checkbox's stateChanged signal to our slot
        connect(checkBox, &QCheckBox::stateChanged, this, &SwitchWidget::onCheckboxStateChanged);

        // Set the initial slider mode text
        onSliderValueChanged(modeSlider->value());
    }

    SwitchWidget::~SwitchWidget()
    {
        delete modeSlider;
        delete sliderLabel1;
        delete sliderLabel2;
        delete sliderLabel3;
        delete checkBox;
        delete infoLabel;
    }

    void SwitchWidget::onSliderValueChanged(int value) {

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

        void SwitchWidget::onCheckboxStateChanged(int state) 
    {
        // Handle checkbox state changes if needed.
        // For instance, you could print the state or do something else.
        if(state == Qt::Checked) 
        {
            // Checkbox is checked: Disable slider and labels
            modeSlider->setEnabled(false);
            sliderLabel1->setEnabled(false);
            sliderLabel2->setEnabled(false);
            sliderLabel3->setEnabled(false);

            modeSlider->setValue(2);

            infoLabel->setText("RI is ON");
            infoLabel->setStyleSheet("QLabel { color : purple; font-weight: bold;}"); // Set text color to red
        } 
        else if(state == Qt::Unchecked) 
        {
            // Checkbox is unchecked: Enable slider and labels
            modeSlider->setEnabled(true);
            sliderLabel1->setEnabled(true);
            sliderLabel2->setEnabled(true);
            sliderLabel3->setEnabled(true);

            modeSlider->setValue(1);

            infoLabel->setText("HI is ON");
            infoLabel->setStyleSheet("QLabel { color : black; font-weight: bold;}"); // Set text color to red
        }
    }
}
