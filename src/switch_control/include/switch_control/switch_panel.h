#ifndef SWITCH_PANEL_H
#define SWITCH_PANEL_H

#include <QVBoxLayout>

#include <rviz_common/panel.hpp>
#include <switch_control/switch_widget.h>

namespace switch_panel
{
    class Switch_widget;

    class SwitchPanel : public rviz_common::Panel
    {
        Q_OBJECT
        public:
            explicit SwitchPanel(QWidget * parent = 0);
            virtual ~SwitchPanel();

            void onInitialize() override;
            void save(rviz_common::Config config) const override;
            void load(const rviz_common::Config &conf) override;
        
        
        private:
            SwitchWidget *_widget;
        
    };
}

#endif // RVIZ_PANEL_H