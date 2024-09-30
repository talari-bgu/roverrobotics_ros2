#include "switch_control/switch_panel.h"

#include <QVBoxLayout>

#include <memory>
#include <vector>
#include <utility>
#include <rviz_common/display_context.hpp>

namespace switch_panel
{
    SwitchPanel::SwitchPanel(QWidget * parent) : Panel(parent)
    {
        _widget = new SwitchWidget(parent);
        QVBoxLayout * layout = new QVBoxLayout;
        layout->addWidget(_widget);
        layout->setContentsMargins(10, 10, 10, 10);
        setLayout(layout);
    }

    SwitchPanel::~SwitchPanel() {}

    void SwitchPanel::save(rviz_common::Config config) const
    {
        Panel::save(config);
    }

    void SwitchPanel::load(const rviz_common::Config &conf)
    {
        Panel::load(conf);
    }

    void SwitchPanel::onInitialize()
    {
        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(switch_panel::SwitchPanel, rviz_common::Panel)