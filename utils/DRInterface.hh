/*
 *  Copyright (C) 2018 João Borrego
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *  
 *      http://www.apache.org/licenses/LICENSE-2.0
 *      
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

/*!
    \file utils/DRInterface.hh
    \brief Domain randomization interface headers

    \author João Borrego : jsbruglie
*/

#ifndef _DOMAIN_RANDOMIZATION_INTERFACE_HH_
#define _DOMAIN_RANDOMIZATION_INTERFACE_HH_

// Gazebo
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <gazebo/physics/bullet/BulletSurfaceParams.hh>
#include <gazebo/physics/ode/ODESurfaceParams.hh>
#include <gazebo/physics/dart/DARTSurfaceParams.hh>

// Sleep
#include <chrono>
#include <thread>

// Custom messages
#include "dr_request.pb.h"
#include "model_cmd.pb.h"
#include "dr_response.pb.h"

// Debug streams
#include "debug.hh"

// Required fields workaround
#include <limits>

/// Declaration for request message type
typedef gap::msgs::DRRequest DRRequest;
/// Declaration for model command message type
typedef gap::msgs::ModelCmd ModelCmdMsg;
    
/// Declaration for response message type
typedef gap::msgs::DRResponse DRResponse;
/// Shared pointer declaration for response message type
typedef const boost::shared_ptr<const gap::msgs::DRResponse>
    DRResponsePtr;

/*
typedef boost::shared_ptr <gazebo::physics::BulletSurfaceParams>
    BulletSurfaceParamsPtr;
typedef boost::shared_ptr <gazebo::physics::DARTSurfaceParams>
    DARTSurfaceParamsPtr;
typedef boost::shared_ptr <gazebo::physics::ODESurfaceParams>
    ODESurfaceParamsPtr;
*/

/// \brief Domain Randomization Plugin Interface
class DRInterface
{

    /// Topic for DRPlugin requests
    public: static const char REQUEST_TOPIC[];
    /// Topic for DRPlugin responses
    public: static const char RESPONSE_TOPIC[];
    /// Topic for outgoing visual requests
    public: static const char VISUAL_TOPIC[];
    /// Position controller type
    public: static const int POSITION;
    /// Velocity controller type
    public: static const int VELOCITY;

    /// Node used for transport
    private: gazebo::transport::NodePtr node;
    /// Publisher to the DRPlugin request topic
    private: gazebo::transport::PublisherPtr pub;
    /// Subscriber to the DRPlugin response topic
    private: gazebo::transport::SubscriberPtr sub;
    /// Publisher to the visual topic
    private: gazebo::transport::PublisherPtr pub_visual;

    /// Topic for DRPlugin requests
    private: std::string req_topic {REQUEST_TOPIC};
    /// Topic for DRPlugin responses
    private: std::string res_topic {RESPONSE_TOPIC};

    /// Global timeout flag
    private: bool wait_done {false};
    /// Mutex for global timeout flag
    private: std::mutex wait_done_mutex;

    /// \brief Constructor
    /// \param req_topic Request topic
    /// \param res_topic Response topic
    public: DRInterface(
        const std::string & req_topic_,
        const std::string & res_topic_);

    /// \brief Constructor with default arguments
    public: DRInterface();

    /// \brief Returns whether to keep waiting for trigger
    /// \param mutex   Mutex that protects trigger variable
    /// \param trigger Trigger boolean variable
    /// \return True as long as trigger is false, false otherwise
    private: static bool waitingTrigger(std::mutex & mutex, bool & trigger);

    /// \brief Creates a domain randomization request
    /// \return Empty request
    public: DRRequest createRequest();

    /// \brief Publishes request
    /// \param msg Domain randomization request
    /// \param blocking Whether to wait for response
    public: void publish(DRRequest & msg, bool blocking=false);

    /// \brief Publishes visual request
    /// \param msg Visual message request
    /// \param blocking Whether to wait for response
    /// \warning Blocking calls not yet implemented for visual messages!
    public: void publish(gazebo::msgs::Visual & msg, bool blocking=false);   

    // Features

    /// \brief Updates physics engine gravity
    /// \param msg Output domain randomization request
    /// \param gravity New gravity vector
    public: void addGravity(DRRequest & msg,
        const ignition::math::Vector3d & gravity);

    /// \brief Updates model scale
    /// \param msg Output domain randomization request
    /// \param model The target model name
    /// \param scale The new model scale
    public: void addModelScale(DRRequest & msg,
        const std::string & model,
        const ignition::math::Vector3d & scale);

    /// \brief Updates link mass
    /// \param msg Output domain randomization request
    /// \param model The target model name
    /// \param link The target link name
    /// \param mass The new link mass
    public: void addLinkMass(DRRequest & msg,
        const std::string & model,
        const std::string & link,
        double mass);

    /// \brief Updates link inertia matrix
    /// \param msg Output domain randomization request
    /// \param model The target model name
    /// \param link The target link name
    /// \param ixx The new Ixx
    /// \param iyy The new Iyy
    /// \param izz The new Izz
    /// \param ixy The new Ixy
    /// \param ixz The new Ixz
    /// \param iyz The new Iyz
    public: void addInertiaMatrix(DRRequest & msg,
        const std::string & model,
        const std::string & link,
        double ixx, double iyy, double izz,
        double ixy, double ixz, double iyz);

    /// \brief Updates collision surface
    /// \param msg Output domain randomization request
    /// \param model The target model
    /// \param link The target link name
    /// \param collision The target collision name
    /// \param surface The new surface parameters
    public: void addSurface(DRRequest & msg,
        const std::string & model,
        const std::string & link,
        const std::string & collision,
        const gazebo::msgs::Surface & surface);

    /// \brief Updates joint
    /// 
    /// \note Does not update value if it is INFINITY
    ///
    /// \param msg Output domain randomization request
    /// \param model The target model name
    /// \param joint The target joint name
    /// \param limit_lower New joint lower limit
    /// \param limit_upper New joint upper limit
    /// \param limit_effort New joint effort limit
    /// \param limit_velocity New joint velocity limit
    /// \param limit_lower New joint damping coefficient (TODO)
    /// \param limit_lower New joint static friction
    public: void addJoint(DRRequest & msg,
        const std::string & model,
        const std::string & joint,
        double limit_lower = INFINITY,
        double limit_upper = INFINITY,
        double limit_effort = INFINITY,
        double limit_velocity = INFINITY,
        double damping = INFINITY,
        double friction = INFINITY);

    /// \brief Updates model controllers
    /// 
    /// \note Does not update value if it is INFINITY
    ///
    /// \param msg Output domain randomization request
    /// \param model The target model name
    /// \param joint The target joint <b>scoped</b> name
    /// \param type Type of controllers (POSITION or VELOCITY)
    /// \param p_gain New P gain
    /// \param i_gain New I gain 
    /// \param d_gain New D gain 
    public: void addModelCmd(DRRequest & msg,
        const std::string & model,
        const std::string & joint,
        int type,
        double p_gain = INFINITY,
        double i_gain = INFINITY,
        double d_gain = INFINITY);

    /// \brief Updates visual color
    /// 
    /// \note Does not update value if it is INFINITY
    ///
    /// \param msg Output visual message request
    /// \param visual The target visual name
    /// \param visual The target visual's parent name
    /// \param ambient The ambient color
    /// \param diffuse The diffuse color
    /// \param emissive The emissive color
    /// \param specular The specular color
    public: void addColors(gazebo::msgs::Visual & msg,
        const std::string & visual,
        const std::string & parent,
        const ignition::math::Color & ambient,
        const ignition::math::Color & diffuse,
        const ignition::math::Color & emissive,
        const ignition::math::Color & specular);

    /// \brief Callback on DRPlugin response
    /// \param _msg Response message
    public: void onResponse(DRResponsePtr & _msg);
};

#endif
