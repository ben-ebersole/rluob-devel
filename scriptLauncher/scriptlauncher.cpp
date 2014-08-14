#include "scriptlauncher.h"

ScriptLauncher::ScriptLauncher(QObject *parent) :
    QObject(parent),
    m_process(new QProcess(this))
{
}

void ScriptLauncher::launchScript()
{
    /*QString program = "gnome-terminal";
    QStringList arguments;
    arguments << "-e" << "/opt/ros/hydro/bin/rosrun assembly add_objects.cpp";
    m_process->start(program,arguments);*/
    //m_process->start("sh /home/bjebersole/scriptLauncher/script/script.sh");
    m_process->start("gnome-terminal -e /opt/ros/hydro/bin/roscore");
}
