#ifndef DEMO_PANEL_H
#define DEMO_PANEL_H

#include <QVBoxLayout>

#include <rviz_common/panel.hpp>
#include <demo_panel/demo_widget.h>

namespace demo_panel
{
    class Demo_widget;

    class DemoPanel : public rviz_common::Panel
    {
        Q_OBJECT
        public:
            explicit DemoPanel(QWidget * parent = 0);
            virtual ~DemoPanel();

            void onInitialize() override;
            void save(rviz_common::Config config) const override;
            void load(const rviz_common::Config &conf) override;
        
        
        private:
            DemoWidget *_widget;
        
    };
}

#endif // RVIZ_PANEL_H