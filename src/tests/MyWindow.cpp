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

#include "MyWindow.h"
#include "simulation/World.h"

using namespace Eigen;

void MyWindow::timeStepping()
{
    VectorXd damping = computeDamping();
    mWorld->getSkeleton(0)->setInternalForces(damping);
    mWorld->step();
}

void MyWindow::draw()
{
	rtql8::simulation::SimWindow::draw();
	Eigen::Vector4d color(0.8, 0.4, 0.4, 1);
	mesh_.draw(color, false, true);
}


#include <iostream>
void MyWindow::keyboard(unsigned char key, int x, int y)
{
    switch(key){
    case '+': // use space key to play or stop the motion
        {
            currentIndex_ ++;
            if (currentIndex_ >= samples_.size()) currentIndex_--;
            samples_[currentIndex_]->LoadIntoLimb(limb_,true);
            break;
        }
    case '-': // use space key to play or stop the motion
        {
            if (currentIndex_ >0) currentIndex_--;
            samples_[currentIndex_]->LoadIntoLimb(limb_,true);
            break;
        }
    case ' ': // use space key to play or stop the motion
        mSimulating = !mSimulating;
        if(mSimulating) {
            mPlay = false;
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        }
        break;
    case 'p': // playBack
        mPlay = !mPlay;
        if (mPlay) {
            mSimulating = false;
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        }
        break;
    case '[': // step backward
        if (!mSimulating) {
            mPlayFrame--;
            if(mPlayFrame < 0)
                mPlayFrame = 0;
            glutPostRedisplay();
        }
        break;
    case ']': // step forwardward
        if (!mSimulating) {
            mPlayFrame++;
            if(mPlayFrame >= mBakedStates.size())
                mPlayFrame = 0;
            glutPostRedisplay();
        }
        break;
    case 'v': // show or hide markers
        mShowMarkers = !mShowMarkers;
        break;
    default:
        Win3D::keyboard(key,x,y);
    }
    glutPostRedisplay();
}

VectorXd MyWindow::computeDamping()
{
    int nDof = mWorld->getSkeleton(0)->getNumDofs();
    VectorXd damping = VectorXd::Zero(nDof);
    // add damping to each joint; twist-dof has smaller damping
    damping = -0.01 * mWorld->getSkeleton(0)->getPoseVelocity();
    for (int i = 0; i < nDof; i++)
        if (i % 3 == 1)
            damping[i] *= 0.1;
    return damping;
}
