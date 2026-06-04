// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef HDE_CONNECTION_MONITOR_H
#define HDE_CONNECTION_MONITOR_H

#include <atomic>

#include <yarp/os/PortInfo.h>
#include <yarp/os/PortReport.h>

namespace hde {

class ConnectionMonitor : public yarp::os::PortReport
{
public:
    bool isConnected() const
    {
        return m_isConnected;
    }

    // yarp::os::PortReport interface — called by YARP from its own thread
    // on both connection and disconnection events.
    // Must not perform any operation on the monitored port (deadlock risk).
    void report(const yarp::os::PortInfo& info) override
    {
        if (info.tag == yarp::os::PortInfo::PORTINFO_CONNECTION && info.incoming) {
            m_isConnected = info.created;
        }
    }

private:
    std::atomic<bool> m_isConnected{false};
};

} // namespace hde

#endif // HDE_CONNECTION_MONITOR_H
