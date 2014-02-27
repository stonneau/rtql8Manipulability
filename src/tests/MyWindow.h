/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>
 * Georgia Tech Graphics Lab
 * 
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * This code incorporates portions of Open Dynamics Engine 
 *     (Copyright (c) 2001-2004, Russell L. Smith. All rights 
 *     reserved.) and portions of FCL (Copyright (c) 2011, Willow 
 *     Garage, Inc. All rights reserved.), which were released under
 *     the same BSD license as below
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _MYWINDOW_
#define _MYWINDOW_

#include "yui/Win3D.h"
#include "simulation/SimWindow.h"
#include "rtql8/geometry/Mesh3DTriangle.h"
#include "rtql8/kinematics/Skeleton.h"
#include "rtql8/kinematics/BodyNode.h"
#include "rtql8Manipulability/sampling/Sample.h"
#include "rtql8Manipulability/sampling/SampleGeneratorVisitor_ABC.h"

#include <vector>

class MyWindow : public rtql8::simulation::SimWindow, public rtql8::kinematics::SampleGeneratorVisitor_ABC
{
 public:
 MyWindow(rtql8::geometry::Mesh3DTriangle& mesh, rtql8::kinematics::BodyNode* limb)
     : SimWindow()
     , mesh_(mesh)
     , limb_(limb)
     , currentIndex_(0)
    {}
    virtual ~MyWindow() {}

    virtual void keyboard(unsigned char key, int x, int y);
    virtual void timeStepping();
    //  virtual void drawSkels();
    //  virtual void displayTimer(int _val);
    virtual void draw();
    //  virtual void keyboard(unsigned char key, int x, int y);
    virtual void Visit(rtql8::kinematics::BodyNode* limb, rtql8::kinematics::Sample& sample)
    {
        if(limb->getName() == limb_->getName())
        {
            samples_.push_back(&sample);
        }
    }

 private:
    Eigen::VectorXd computeDamping(); 
    rtql8::geometry::Mesh3DTriangle& mesh_; 
    std::vector<const rtql8::kinematics::Sample*> samples_;
    rtql8::kinematics::BodyNode* limb_;
    int currentIndex_;
};

#endif
