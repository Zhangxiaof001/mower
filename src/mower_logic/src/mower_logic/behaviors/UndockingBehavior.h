// Created by Clemens Elflein on 2/21/22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//
// This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
//
// Feel free to use the design in your private/educational projects, but don't try to sell the design or products based on it without getting my consent first.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//
#ifndef SRC_UNDOCKINGBEHAVIOR_H
#define SRC_UNDOCKINGBEHAVIOR_H

#include <actionlib/client/simple_action_client.h>
#include <mbf_msgs/ExePathAction.h>
#include <nav_msgs/Odometry.h>
#include "Behavior.h"
#include "IdleBehavior.h"
#include "DockingBehavior.h"
#include "ros/ros.h"
#include <tf2/LinearMath/Transform.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "MowingBehavior.h"


class UndockingBehavior : public Behavior {
public:
    static UndockingBehavior INSTANCE;
    static UndockingBehavior RETRY_INSTANCE;

    // 构造函数
    UndockingBehavior(Behavior* nextBehavior);
private:
    Behavior* nextBehavior;  // 下一个状态，在构造函数中赋值
    geometry_msgs::PoseStamped docking_pose_stamped;  // 对接点, 好像没有用上？
    // 是否需要gps
    bool gpsRequired;

    bool waitForGPS();


public:
    std::string state_name() override;

    Behavior *execute() override;

    void enter() override;

    void exit() override;

    void reset() override;

    bool needs_gps() override;

    bool mower_enabled() override;

    void command_home() override;

    void command_start() override;

    void command_s1() override;

    void command_s2() override;

    bool redirect_joystick() override;
};


#endif //SRC_UNDOCKINGBEHAVIOR_H
