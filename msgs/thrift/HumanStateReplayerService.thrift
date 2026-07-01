// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

namespace yarp hde.msgs

service HumanStateReplayerService {
    bool start();
    bool pause();
    bool reset();
}
