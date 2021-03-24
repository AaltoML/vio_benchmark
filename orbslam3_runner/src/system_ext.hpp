/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
// #include <openssl/md5.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/string.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

#include <functional>

namespace ORB_SLAM3
{

Verbose::eLevel Verbose::th = Verbose::VERBOSITY_NORMAL;

class SystemExt : public System {
public:

    using System::System;

    // Exports final estimated trajectory to the given function
    void ExportMap(std::function<void(
        double time,
        double posX, double posY, double posZ,
        double rotW, double rotX, double rotY, double rotZ
        )> onPose) {

        /*if(mSensor==MONOCULAR)
        {
            cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." << endl;
            return;
        }*/

        vector<Map*> vpMaps = mpAtlas->GetAllMaps();
        Map* pBiggerMap;
        int numMaxKFs = 0;
        for(Map* pMap :vpMaps)
        {
            if(pMap->GetAllKeyFrames().size() > numMaxKFs)
            {
                numMaxKFs = pMap->GetAllKeyFrames().size();
                pBiggerMap = pMap;
            }
        }

        vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
        sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        cv::Mat Twb; // Can be word to cam0 or world to b dependingo on IMU or not.
        if (mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO)
            Twb = vpKFs[0]->GetImuPose();
        else
            Twb = vpKFs[0]->GetPoseInverse();

        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
        // We need to get first the keyframe pose and then concatenate the relative transformation.
        // Frames not localized (tracking failure) are not saved.

        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
        // which is true when tracking failed (lbL).
        list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        list<bool>::iterator lbL = mpTracker->mlbLost.begin();

        //cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << endl;
        //cout << "size mlRelativeFramePoses: " << mpTracker->mlRelativeFramePoses.size() << endl;
        //cout << "size mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << endl;
        //cout << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;


        for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
            lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
        {
            //cout << "1" << endl;
            if(*lbL)
                continue;


            KeyFrame* pKF = *lRit;
            //cout << "KF: " << pKF->mnId << endl;

            cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

            /*cout << "2" << endl;
            cout << "KF id: " << pKF->mnId << endl;*/

            // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
            if (!pKF)
                continue;

            //cout << "2.5" << endl;

            while(pKF->isBad())
            {
                //cout << " 2.bad" << endl;
                Trw = Trw*pKF->mTcp;
                pKF = pKF->GetParent();
                //cout << "--Parent KF: " << pKF->mnId << endl;
            }

            if(!pKF || pKF->GetMap() != pBiggerMap)
            {
                //cout << "--Parent KF is from another map" << endl;
                /*if(pKF)
                    cout << "--Parent KF " << pKF->mnId << " is from another map " << pKF->GetMap()->GetId() << endl;*/
                continue;
            }

            //cout << "3" << endl;

            Trw = Trw*pKF->GetPose()*Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

            // cout << "4" << endl;

            if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO) {
                cv::Mat Tbw = pKF->mImuCalib.Tbc*(*lit)*Trw;
                cv::Mat Rwb = Tbw.rowRange(0,3).colRange(0,3).t();
                cv::Mat twb = -Rwb*Tbw.rowRange(0,3).col(3);
                vector<float> q = Converter::toQuaternion(Rwb);
                onPose(
                    *lT,
                    twb.at<float>(0), twb.at<float>(1), twb.at<float>(2),
                    q[3], q[0], q[1], q[2]);
            } else {
                cv::Mat Tcw = (*lit)*Trw;
                cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
                cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
                vector<float> q = Converter::toQuaternion(Rwc);
                onPose(
                    *lT,
                    twc.at<float>(0), twc.at<float>(1), twc.at<float>(2),
                    q[3], q[0], q[1], q[2]);
            }

            // cout << "5" << endl;
        }
        //cout << "end saving trajectory" << endl;
    }
};


} // namespace
