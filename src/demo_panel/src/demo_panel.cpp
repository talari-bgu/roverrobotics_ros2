#include "demo_panel/demo_panel.h"

#include <QVBoxLayout>

#include <memory>
#include <vector>
#include <utility>
#include <rviz_common/display_context.hpp>

namespace demo_panel
{
    DemoPanel::DemoPanel(QWidget * parent) : Panel(parent)
    {
        _widget = new DemoWidget(parent);
        QVBoxLayout * layout = new QVBoxLayout;
        layout->addWidget(_widget);
        layout->setContentsMargins(10, 10, 10, 10);
        setLayout(layout);
    }

    DemoPanel::~DemoPanel() {}

    void DemoPanel::save(rviz_common::Config config) const
    {
        Panel::save(config);
    }

    void DemoPanel::load(const rviz_common::Config &conf)
    {
        Panel::load(conf);
    }

    void DemoPanel::onInitialize()
    {
        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(demo_panel::DemoPanel, rviz_common::Panel)