/* Created  :   Linhui
 * Date     :   2016-05-17
 * Usage    :   Definition of member function of class VELODYNE_PARSER
*/
#include "velodyne_points_publish/velodyne_points_parser.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>
#include <fstream>
#include <sstream>
#include <cassert>
#include <cmath>
#include <cctype>
using namespace std;

const float LIDAR_SPIN_CYCLE = 0.1;


/* HDL64_INTRINSIC_PARA part:
 *  constructor
*/
HDL64_INTRINSIC_PARA::HDL64_INTRINSIC_PARA()
{
    memset(hCorrection, 0, sizeof(float)*HDL64_BEAM_NUM);
    memset(vCorrection, 0, sizeof(float)*HDL64_BEAM_NUM);
    memset(distCorrection, 0, sizeof(float)*HDL64_BEAM_NUM);
    memset(vOffsetCorrection, 0, sizeof(float)*HDL64_BEAM_NUM);
    memset(hOffsetCorrection, 0, sizeof(float)*HDL64_BEAM_NUM);
    memset(focalD, 0, sizeof(float)*HDL64_BEAM_NUM);
    memset(focalS, 0, sizeof(float)*HDL64_BEAM_NUM);
    memset(minIntensity, 0, sizeof(float)*HDL64_BEAM_NUM);
    memset(maxIntensity, 0, sizeof(float)*HDL64_BEAM_NUM);

    memset(angleV, 0, sizeof(float)*HDL64_BEAM_NUM);
    memset(sin_angleH, 0, sizeof(float)*36000);
    memset(cos_angleH, 0, sizeof(float)*36000);
    memset(sin_angleV, 0, sizeof(float)*HDL64_BEAM_NUM);
    memset(cos_angleV, 0, sizeof(float)*HDL64_BEAM_NUM);

    memset(beam_order, 0, sizeof(int)*HDL64_BEAM_NUM);
    memset(beam_reverse_order, 0, sizeof(int)*HDL64_BEAM_NUM);
}


/* HDL32_INTRINSIC_PARA part:
 *  constructor
*/
HDL32_INTRINSIC_PARA::HDL32_INTRINSIC_PARA()
{
    memset(angleV, 0, sizeof(float)*HDL32_BEAM_NUM);
    memset(beam_order, 0, sizeof(int)*HDL32_BEAM_NUM);
    memset(beam_reverse_order, 0, sizeof(int)*HDL32_BEAM_NUM);
    memset(sin_angleH, 0, sizeof(float)*36000);
    memset(cos_angleH, 0, sizeof(float)*36000);
    memset(sin_angleV, 0, sizeof(float)*HDL32_BEAM_NUM);
    memset(cos_angleV, 0, sizeof(float)*HDL32_BEAM_NUM);
}

/* VLP16_INTRINSIC_PARA part:
 *  constructor
*/
VLP16_INTRINSIC_PARA::VLP16_INTRINSIC_PARA()
{
    memset(angleV, 0, sizeof(float)*VLP16_BEAM_NUM);
    memset(sin_angleV, 0, sizeof(float)*VLP16_BEAM_NUM);
    memset(cos_angleV, 0, sizeof(float)*VLP16_BEAM_NUM);
    memset(sin_angleH, 0, sizeof(float)*36000);
    memset(cos_angleH, 0, sizeof(float)*36000);
    memset(beam_point_num, 0, sizeof(int)*VLP16_BEAM_NUM);
}

/* LIDAR_EXTRINSIC_PARA part:
 *  constructor
*/
LIDAR_EXTRINSIC_PARA::LIDAR_EXTRINSIC_PARA()
{
    memset(R, 0, sizeof(float)*3*3);
    memset(T, 0, sizeof(float)*3);
}


PARA_TABLE::PARA_TABLE()
{
    para_base_dir = "/root/ZJUALV/bin/parameters";
}

void PARA_TABLE::print_base_dir()
{
    cout<<"---------------- NOTICE !!! ----------------"<<endl<<endl;
    cout<<" The parameters for lidar32 & lidar16 calibration is default located in :"<<endl;
    cout<<"\t\""<<para_base_dir<<"\""<<endl<<endl;
    cout<<"-------------- END NOTICE !!! --------------"<<endl<<endl;
}

PARA_TABLE::~PARA_TABLE(){}

/* VELODYNE_PARSER part:
 *  constructor & destructor
 *  initiation of parameters
 *  setup
 *  clear_points
*/
VELODYNE_PARSER::VELODYNE_PARSER()
{
    memset(lidar64_pointcloud, 0, sizeof(mPoint3f)*HDL64_BEAM_NUM*HDL64_BEAM_POINTSIZE);
    memset(lidar32_pointcloud, 0, sizeof(mPoint3f)*HDL32_BEAM_NUM*HDL32_BEAM_POINTSIZE);
    memset(lidar16_pointcloud_L, 0, sizeof(mPoint3f)*VLP16_BEAM_NUM*VLP16_BEAM_POINTSIZE);
    memset(lidar16_pointcloud_R, 0, sizeof(mPoint3f)*VLP16_BEAM_NUM*VLP16_BEAM_POINTSIZE);

    lidar32_pointNum = 0;
    lidar64_pointNum = 0;
}

VELODYNE_PARSER::~VELODYNE_PARSER(){}
bool VELODYNE_PARSER::init_para()
{
    if(init_lidar16_para() && init_lidar32_para() && init_lidar64_para())
        return true;
    else
        return false;
}

bool CMP(pair<float, int> a, pair<float, int> b)
{
    return a.first < b.first;
}

bool VELODYNE_PARSER::arrange_lidar32_beam_order()
{
    vector<pair<float, int> > beams;
    for(int i=0; i<HDL32_BEAM_NUM; i++)
    {
        beams.push_back(pair<float, int>(para_table.lidar32_inpara.angleV[i], i));
    }
    sort(beams.begin(), beams.end(), CMP);
    for(int i=0; i<beams.size(); i++)
	{
        para_table.lidar32_inpara.beam_order[i] = beams[i].second;
		para_table.lidar32_inpara.beam_reverse_order[beams[i].second] = i;
	}
}

bool VELODYNE_PARSER::arrange_lidar64_beam_order()
{
    vector<pair<float, int> > beams;
    for(int i=0; i<HDL64_BEAM_NUM; i++)
    {
        beams.push_back(pair<float, int>(para_table.lidar64_inpara.angleV[i], i));
    }
    sort(beams.begin(), beams.end(), CMP);
    for(int i=0; i<beams.size(); i++)
    {
        para_table.lidar64_inpara.beam_order[i] = beams[i].second;
        para_table.lidar64_inpara.beam_reverse_order[beams[i].second] = i;
    }
}


bool VELODYNE_PARSER::init_lidar64_para()
{
    string lidar64para_filename = para_table.para_base_dir + "/lidar64para.ini";
    ifstream infile;
    stringstream sline;
    string line;

    for(int k = 0; k<36000; k++)
    {
        para_table.lidar64_inpara.sin_angleH[k] = (float)sin((double)k/18000.0*M_PI);
        para_table.lidar64_inpara.cos_angleH[k] = (float)cos((double)k/18000.0*M_PI);
    }

    infile.open(lidar64para_filename, ios::in);
    if(!infile)
    {
        cerr<<"***Error: can't open hdl64 para file \""<<lidar64para_filename<<"\""<<endl;
        return false;
    }

    sline.str("");
    sline.clear();
    line.clear();
    while(getline(infile, line))
    {
        if(line.empty())
            continue;
        if(line[0] == '#')      // '#' means comment
            continue;
        if(line[0] == '[')      // '[' means this is a flag line
        {
            string flag;
            for(auto c : line)
            {
                if(c != '[' && c != ']' && c != '\\')
                    flag = flag + c;
                if(c == '\\')
                    break;
            }
            if(flag == "Correction")
            {
                for(int i=0; i<64; i++)
                {
                    line.clear();
                    getline(infile, line);
                    sline.str("");
                    sline.clear();
                    sline<<line;

                    sline >>para_table.lidar64_inpara.hCorrection[i]
                          >>para_table.lidar64_inpara.vCorrection[i]
                          >>para_table.lidar64_inpara.distCorrection[i]
                          >>para_table.lidar64_inpara.vOffsetCorrection[i]
                          >>para_table.lidar64_inpara.hOffsetCorrection[i]
                          >>para_table.lidar64_inpara.focalD[i]
                          >>para_table.lidar64_inpara.focalS[i]
                          >>para_table.lidar64_inpara.minIntensity[i]
                          >>para_table.lidar64_inpara.maxIntensity[i];

                    para_table.lidar64_inpara.angleV[i] = para_table.lidar64_inpara.vCorrection[i];
                    para_table.lidar64_inpara.sin_angleV[i] = (float)sin(para_table.lidar64_inpara.angleV[i]/180.0*M_PI);
                    para_table.lidar64_inpara.cos_angleV[i] = (float)cos(para_table.lidar64_inpara.angleV[i]/180.0*M_PI);
                }
                arrange_lidar64_beam_order();
            }
            else if(flag == "Rotation")
            {
                for(int i=0; i<3; i++)
                {
                    line.clear();
                    getline(infile, line);
                    sline.str("");
                    sline.clear();
                    sline<<line;
                    sline>>para_table.lidar64_expara.R[i][0]>>para_table.lidar64_expara.R[i][1]>>para_table.lidar64_expara.R[i][2];
                }
            }
            else if(flag == "Translation")
            {
                line.clear();
                getline(infile, line);
                sline.str("");
                sline.clear();
                sline<<line;
                sline>>para_table.lidar64_expara.T[0]>>para_table.lidar64_expara.T[1]>>para_table.lidar64_expara.T[2];
            }
        }
        line.clear();
    }
    infile.close();
    return true;
}


bool VELODYNE_PARSER::init_lidar32_para()
{
    string lidar32para_filename = para_table.para_base_dir + "/lidar32para.ini";
    ifstream infile;
    stringstream sline;
    string line;

    for(int k = 0; k<36000; k++)
    {
        para_table.lidar32_inpara.sin_angleH[k] = (float)sin((double)k/18000.0*M_PI);
        para_table.lidar32_inpara.cos_angleH[k] = (float)cos((double)k/18000.0*M_PI);
    }

    infile.open(lidar32para_filename, ios::in);
    if(!infile)
    {
        cerr<<"***Error: can't open hdl32 para file \""<<lidar32para_filename<<"\""<<endl;
        return false;
    }

    sline.str("");
    sline.clear();
    line.clear();
    while(getline(infile, line))
    {
        if(line.empty())
            continue;
        if(line[0] == '#')      // '#' means comment
            continue;
        if(line[0] == '[')      // '[' means this is a flag line
        {
            string flag;
            for(auto c : line)
            {
                if(c != '[' && c != ']' && c != '\\')
                    flag = flag + c;
                if(c == '\\')
                    break;
            }
            if(flag == "Angle_V")
            {
                for(int i=0; i<32; i++)
                {
                    line.clear();
                    getline(infile, line);
                    sline.str("");
                    sline.clear();
                    sline<<line;
                    sline>>para_table.lidar32_inpara.angleV[i];

                    para_table.lidar32_inpara.sin_angleV[i] = (float)sin(para_table.lidar32_inpara.angleV[i]/180.0*M_PI);
                    para_table.lidar32_inpara.cos_angleV[i] = (float)cos(para_table.lidar32_inpara.angleV[i]/180.0*M_PI);
                }
                arrange_lidar32_beam_order();
            }
            else if(flag == "Rotation")
            {
                for(int i=0; i<3; i++)
                {
                    line.clear();
                    getline(infile, line);
                    sline.str("");
                    sline.clear();
                    sline<<line;
                    sline>>para_table.lidar32_expara.R[i][0]>>para_table.lidar32_expara.R[i][1]>>para_table.lidar32_expara.R[i][2];
                }
            }
            else if(flag == "Translation")
            {
                line.clear();
                getline(infile, line);
                sline.str("");
                sline.clear();
                sline<<line;
                sline>>para_table.lidar32_expara.T[0]>>para_table.lidar32_expara.T[1]>>para_table.lidar32_expara.T[2];
            }
        }
        line.clear();
    }
    infile.close();
    return true;
}


bool VELODYNE_PARSER::init_lidar16_para()
{
    /* init intrinsic para table
     *
    */
    string lidar16para_L_filename = para_table.para_base_dir + "/lidar16para_L.ini";
    string lidar16para_R_filename = para_table.para_base_dir + "/lidar16para_R.ini";
    ifstream infile;
    stringstream sline;
    string line;

    // left
    for(int k = 0; k<36000; k++)
    {
        para_table.lidar16_inpara_L.sin_angleH[k] = (float)sin((double)k/18000.0*M_PI);
        para_table.lidar16_inpara_L.cos_angleH[k] = (float)cos((double)k/18000.0*M_PI);
    }

    infile.open(lidar16para_L_filename, ios::in);
    if(!infile)
    {
        cerr<<"***Error: can't open vlp16-L para file \""<<lidar16para_L_filename<<"\""<<endl;
        return false;
    }

    sline.str("");
    sline.clear();
    line.clear();
    while(getline(infile, line))
    {
        if(line.empty())
            continue;
        if(line[0] == '#')      // '#' means comment
            continue;
        if(line[0] == '[')      // '[' means this is a flag line
        {
            string flag;
            for(auto c : line)
            {
                if(c != '[' && c != ']' && c != '\\')
                    flag = flag + c;
                if(c == '\\')
                    break;
            }
            if(flag == "Angle_V")
            {
                for(int i=0; i<VLP16_BEAM_NUM; i++)
                {
                    line.clear();   // remember to clear!!!!!
                    getline(infile, line);
                    sline.str("");
                    sline.clear();  // remember to clear!!!!!
                    sline<<line;
                    sline>>para_table.lidar16_inpara_L.angleV[i];
                    para_table.lidar16_inpara_L.sin_angleV[i] = (float)sin(para_table.lidar16_inpara_L.angleV[i]/180.0*M_PI);
                    para_table.lidar16_inpara_L.cos_angleV[i] = (float)cos(para_table.lidar16_inpara_L.angleV[i]/180.0*M_PI);
                }
            }

            else if(flag == "Rotation")
            {
                for(int i=0; i<3; i++)
                {
                    line.clear();
                    getline(infile, line);
                    sline.str("");  // remember to clear!!!!!
                    sline.clear();
                    sline<<line;
                    sline>>para_table.lidar16_expara_L.R[i][0]>>para_table.lidar16_expara_L.R[i][1]>>para_table.lidar16_expara_L.R[i][2];
                }
            }
            else if(flag == "Translation")
            {
                line.clear();
                getline(infile, line);
                sline.str("");  // remember to clear!!!!!
                sline.clear();
                sline<<line;
                sline>>para_table.lidar16_expara_L.T[0]>>para_table.lidar16_expara_L.T[1]>>para_table.lidar16_expara_L.T[2];
                break;
            }
        }
        line.clear();
    }
    infile.close();

    // right
    for(int k = 0; k<36000; k++)
    {
        para_table.lidar16_inpara_R.sin_angleH[k] = (float)sin((double)k/18000.0*M_PI);
        para_table.lidar16_inpara_R.cos_angleH[k] = (float)cos((double)k/18000.0*M_PI);
    }
    infile.open(lidar16para_R_filename, ios::in);
    if(!infile)
    {
        cerr<<"***Error: can't open vlp16-R para file \""<<lidar16para_R_filename<<"\""<<endl;
        return false;
    }

    sline.str("");
    sline.clear();
    line.clear();
    while(getline(infile, line))
    {
        if(line.empty())
            continue;
        if(line[0] == '#')      // '#' means comment
            continue;
        if(line[0] == '[')      // '[' means this is a flag line
        {
            string flag;
            for(auto c : line)
            {
                if(c != '[' && c != ']' && c != '\\')
                    flag = flag + c;
                if(c == '\\')
                    break;
            }

            if(flag == "Angle_V")
            {
                for(int i=0; i<VLP16_BEAM_NUM; i++)
                {
                    line.clear();   // remember to clear!!!!!
                    getline(infile, line);
                    sline.str("");  // remember to clear!!!!!
                    sline.clear();
                    sline<<line;
                    sline>>para_table.lidar16_inpara_R.angleV[i];
                    para_table.lidar16_inpara_R.sin_angleV[i] = (float)sin(para_table.lidar16_inpara_R.angleV[i]/180.0*M_PI);
                    para_table.lidar16_inpara_R.cos_angleV[i] = (float)cos(para_table.lidar16_inpara_R.angleV[i]/180.0*M_PI);
                }
            }
            else if(flag == "Rotation")
            {
                for(int i=0; i<3; i++)
                {
                    line.clear();
                    getline(infile, line);
                    sline.str("");  // remember to clear!!!!!
                    sline.clear();
                    sline<<line;
                    sline>>para_table.lidar16_expara_R.R[i][0]>>para_table.lidar16_expara_R.R[i][1]>>para_table.lidar16_expara_R.R[i][2];
                }
            }
            else if(flag == "Translation")
            {
                line.clear();
                getline(infile, line);
                sline.str("");  // remember to clear!!!!!
                sline.clear();
                sline<<line;
                sline>>para_table.lidar16_expara_R.T[0]>>para_table.lidar16_expara_R.T[1]>>para_table.lidar16_expara_R.T[2];
            }

        }
        line.clear();
    }
    infile.close();


    return true;
}


void VELODYNE_PARSER::clear_points()
{
    memset(lidar16_pointcloud_L, 0, sizeof(mPoint3f)*VLP16_BEAM_NUM*VLP16_BEAM_POINTSIZE);
    memset(lidar16_pointcloud_R, 0, sizeof(mPoint3f)*VLP16_BEAM_NUM*VLP16_BEAM_POINTSIZE);
    memset(lidar32_pointcloud, 0, sizeof(mPoint3f)*HDL32_BEAM_NUM*HDL32_BEAM_POINTSIZE);
    memset(lidar64_pointcloud, 0, sizeof(mPoint3f)*HDL64_BEAM_NUM*HDL64_BEAM_POINTSIZE);

    memset(para_table.lidar16_inpara_L.beam_point_num, 0, sizeof(int)*VLP16_BEAM_NUM);
    memset(para_table.lidar16_inpara_R.beam_point_num, 0, sizeof(int)*VLP16_BEAM_NUM);

    lidar32_pointNum = 0;
    lidar64_pointNum = 0;
}

bool VELODYNE_PARSER::parse_lidar16_process(u_int8_t *cache_L, u_int8_t *cache_R)
{
    /* parse left point cloud
     * 参考VLP16雷达用户手册
    */
    int packet_cnt = 0;
    u_int8_t *fp = cache_L + VLP16_PACKET_SIZE*packet_cnt;
    u_int8_t *fq = fp + 1;
    u_int16_t val = (*fp)*256 + *fq;

    float *cos_angH_L = para_table.lidar16_inpara_L.cos_angleH;
    float *sin_angH_L = para_table.lidar16_inpara_L.sin_angleH;
    float *cos_angV_L = para_table.lidar16_inpara_L.cos_angleV;
    float *sin_angV_L = para_table.lidar16_inpara_L.sin_angleV;
    float *angleV_L = para_table.lidar16_inpara_L.angleV;

    int idx = 0;
    const int BEAM_ORDER[16] = {0, 8, 1, 9, 2, 10, 3, 11, 4, 12, 5, 13, 6, 14, 7, 15};
    u_int32_t start_angleH;
    bool start_angleH_generated = false;
    while(val == 0xffee)
    {
        // parse 1206 bytes
        u_int32_t angleH[24];
        u_int8_t *pAngL = fq + 1;
        u_int8_t *pAngH = pAngL + 1;
        for(int i=0; i<12; i++)                     // read the actual angle
        {
            angleH[2*i] = (*pAngH)*256 + (*pAngL);  // little endian mode
            pAngL += 100;
            pAngH += 100;
        }
        for(int i=0; i<11; i++)                     // interpolate angle
        {
            if(angleH[2*i] < angleH[2*i+2])
                angleH[2*i+1] = ((angleH[2*i] + angleH[2*i+2])/2) % 36000;
            else
            {
                angleH[2*i+1] = ((angleH[2*i] + 36000 + angleH[2*i+2])/2) % 36000;
            }
        }
        if(angleH[20] < angleH[22])
            angleH[23] = (angleH[22] + (angleH[22] - angleH[20])/2) % 36000;
        else
            angleH[23] = (angleH[22] + (angleH[22] + 36000 - angleH[20])/2) % 36000;

        // 确定帧开始的角度
        if(!start_angleH_generated)
		{
			start_angleH = angleH[0];
      std::cout<<"start_angleH: "<<(float)start_angleH/100.0<<std::endl;
			start_angleH_generated = true;
		}
        // generate point position
        for(int i=0; i<12; i++)
        {
            u_int8_t *pL = fp + 100*i + 4;
            u_int8_t *pH = pL + 1;
            u_int8_t *pVal = pH + 1;

            for(int ind=0; ind<VLP16_BEAM_NUM; ind++)
            {
                int j = BEAM_ORDER[ind];    // 按角度排序的实际beam序号
                int distance = ((*pH)*256 + (*pL))*2;       // in mm
                if(distance >= VLP16_VALID_RADIUS_L && !(angleH[2*i] >= VLP16_INVALID_ANGLE_LL && angleH[2*i] <= VLP16_INVALID_ANGLE_LH))
                {
                    lidar16_pointcloud_L[j][idx].valid = true;
                    lidar16_pointcloud_L[j][idx].distance = distance;
                    lidar16_pointcloud_L[j][idx].angleH = angleH[2*i];
                    lidar16_pointcloud_L[j][idx].angleV = int16_t(angleV_L[j]*100);
                    lidar16_pointcloud_L[j][idx].x = (float)distance/10.0 * cos_angV_L[ind] * sin_angH_L[angleH[2*i]];
                    lidar16_pointcloud_L[j][idx].y = (float)distance/10.0 * cos_angV_L[ind] * cos_angH_L[angleH[2*i]];
                    lidar16_pointcloud_L[j][idx].z = (float)distance/10.0 * sin_angV_L[ind];
                    float angleH_dis = float(angleH[2*i]) - float(start_angleH);
                    if(angleH_dis < 0.0)
	                    angleH_dis += 36000.0;
                    float time_ratio = LIDAR_SPIN_CYCLE * angleH_dis / 36000.0;
                    lidar16_pointcloud_L[j][idx].intensity = float(j) + time_ratio;
                }
                pL += 3;
                pH += 3;
                pVal += 3;
            }
            idx++;

            for(int ind=0; ind<VLP16_BEAM_NUM; ind++)
            {
                int j = BEAM_ORDER[ind];    // 按角度排序的实际beam序号
                int distance = ((*pH)*256 + (*pL))*2;       // in mm
                if(distance >= VLP16_VALID_RADIUS_L && !(angleH[2*i+1] >= VLP16_INVALID_ANGLE_LL && angleH[2*i+1] <= VLP16_INVALID_ANGLE_LH))
                {
                    lidar16_pointcloud_L[j][idx].valid = true;
                    lidar16_pointcloud_L[j][idx].distance = distance;
                    lidar16_pointcloud_L[j][idx].angleH = angleH[2*i+1];
                    lidar16_pointcloud_L[j][idx].angleV = int16_t(angleV_L[j]*100);
                    lidar16_pointcloud_L[j][idx].x = (float)distance/10.0 * cos_angV_L[ind] * sin_angH_L[angleH[2*i+1]];
                    lidar16_pointcloud_L[j][idx].y = (float)distance/10.0 * cos_angV_L[ind] * cos_angH_L[angleH[2*i+1]];
                    lidar16_pointcloud_L[j][idx].z = (float)distance/10.0 * sin_angV_L[ind];
                    float angleH_dis = float(angleH[2*i+1]) - float(start_angleH);
                    if(angleH_dis < 0.0)
	                    angleH_dis += 36000.0;
                    float time_ratio = LIDAR_SPIN_CYCLE * angleH_dis / 36000.0;
                    lidar16_pointcloud_L[j][idx].intensity = float(j) + time_ratio;
                }
                pL += 3;
                pH += 3;
                pVal += 3;
            }
            idx++;
        }

        // next packet
        packet_cnt++;
        if(packet_cnt > VLP16_PACKET_NUM - 1)       // in case of exceeding memory
            break;
        fp = cache_L + VLP16_PACKET_SIZE*packet_cnt;
        fq = fp + 1;
        val = (*fp)*256 + *fq;
    }


    /* parse right point cloud
     *
    */
    packet_cnt = 0;
    fp = cache_R + VLP16_PACKET_SIZE*packet_cnt;
    fq = fp + 1;
    val = (*fp)*256 + *fq;

    float *cos_angH_R = para_table.lidar16_inpara_R.cos_angleH;
    float *sin_angH_R = para_table.lidar16_inpara_R.sin_angleH;
    float *cos_angV_R = para_table.lidar16_inpara_R.cos_angleV;
    float *sin_angV_R = para_table.lidar16_inpara_R.sin_angleV;
    float *angleV_R = para_table.lidar16_inpara_R.angleV;

    idx = 0;
    start_angleH_generated = false;
    while(val == 0xffee)
    {
        // parse 1206 bytes
        u_int32_t angleH[24];
        u_int8_t *pAngL = fq + 1;
        u_int8_t *pAngH = pAngL + 1;
        for(int i=0; i<12; i++)                     // read the actual angle
        {
            angleH[2*i] = (*pAngH)*256 + (*pAngL);  // little endian mode
            pAngL += 100;
            pAngH += 100;
        }
        for(int i=0; i<11; i++)                     // interpolate angle
        {
            if(angleH[2*i] < angleH[2*i+2])
                angleH[2*i+1] = ((angleH[2*i] + angleH[2*i+2])/2) % 36000;
            else
            {
                angleH[2*i+1] = ((angleH[2*i] + 36000 + angleH[2*i+2])/2) % 36000;
            }
        }
        if(angleH[20] < angleH[22])
            angleH[23] = (angleH[22] + (angleH[22] - angleH[20])/2) % 36000;
        else
            angleH[23] = (angleH[22] + (angleH[22] + 36000 - angleH[20])/2) % 36000;

        // 确定帧开始的角度
        if(!start_angleH_generated)
		{
			start_angleH = angleH[0];
			start_angleH_generated = true;
		}

        // generate point position
        for(int i=0; i<12; i++)
        {
            u_int8_t *pL = fp + 100*i + 4;
            u_int8_t *pH = pL + 1;
            u_int8_t *pVal = pH + 1;

            for(int ind=0; ind<VLP16_BEAM_NUM; ind++)
            {
                int j = BEAM_ORDER[ind];    // 按角度排序的实际beam序号
                int distance = ((*pH)*256 + (*pL))*2;       // in mm
                if(distance >= VLP16_VALID_RADIUS_L && !(angleH[2*i] >= VLP16_INVALID_ANGLE_RL && angleH[2*i] <= VLP16_INVALID_ANGLE_RH))
                {
                    lidar16_pointcloud_R[j][idx].valid = true;
                    lidar16_pointcloud_R[j][idx].distance = distance;
                    lidar16_pointcloud_R[j][idx].angleH = angleH[2*i];
                    lidar16_pointcloud_R[j][idx].angleV = int16_t(angleV_R[j]*100);
                    lidar16_pointcloud_R[j][idx].x = (float)distance/10.0 * cos_angV_R[ind] * sin_angH_R[angleH[2*i]];
                    lidar16_pointcloud_R[j][idx].y = (float)distance/10.0 * cos_angV_R[ind] * cos_angH_R[angleH[2*i]];
                    lidar16_pointcloud_R[j][idx].z = (float)distance/10.0 * sin_angV_R[ind];
                    float angleH_dis = float(angleH[2*i]) - float(start_angleH);
                    if(angleH_dis < 0.0)
	                    angleH_dis += 36000.0;
                    float time_ratio = LIDAR_SPIN_CYCLE * angleH_dis / 36000.0;
                    lidar16_pointcloud_R[j][idx].intensity = float(j) + time_ratio;
                }
                pL += 3;
                pH += 3;
                pVal += 3;
            }
            idx++;

            for(int ind=0; ind<VLP16_BEAM_NUM; ind++)
            {
                int j = BEAM_ORDER[ind];    // 按角度排序的实际beam序号
                int distance = ((*pH)*256 + (*pL))*2;       // in mm
                if(distance >= VLP16_VALID_RADIUS_L && !(angleH[2*i+1] >= VLP16_INVALID_ANGLE_RL && angleH[2*i+1] <= VLP16_INVALID_ANGLE_RH))
                {
                    lidar16_pointcloud_R[j][idx].valid = true;
                    lidar16_pointcloud_R[j][idx].distance = distance;
                    lidar16_pointcloud_R[j][idx].angleH = angleH[2*i+1];
                    lidar16_pointcloud_R[j][idx].angleV = int16_t(angleV_R[j]*100);
                    lidar16_pointcloud_R[j][idx].x = (float)distance/10.0 * cos_angV_R[ind] * sin_angH_R[angleH[2*i+1]];
                    lidar16_pointcloud_R[j][idx].y = (float)distance/10.0 * cos_angV_R[ind] * cos_angH_R[angleH[2*i+1]];
                    lidar16_pointcloud_R[j][idx].z = (float)distance/10.0 * sin_angV_R[ind];
                    float angleH_dis = float(angleH[2*i+1]) - float(start_angleH);
                    if(angleH_dis < 0.0)
	                    angleH_dis += 36000.0;
                    float time_ratio = LIDAR_SPIN_CYCLE * angleH_dis / 36000.0;
                    lidar16_pointcloud_R[j][idx].intensity = float(j) + time_ratio;
                }
                pL += 3;
                pH += 3;
                pVal += 3;
            }
            idx++;
        }

        // next packet
        packet_cnt++;
        if(packet_cnt > VLP16_PACKET_NUM - 1)       // in case of exceeding memory
            break;
        fp = cache_R + VLP16_PACKET_SIZE*packet_cnt;
        fq = fp + 1;
        val = (*fp)*256 + *fq;
    }
    return true;
}

bool VELODYNE_PARSER::parse_lidar16_data(const string filename_L, const string filename_R)
{
    char file_L[256];
    char file_R[256];
    strcpy(file_L, filename_L.c_str());
    strcpy(file_R, filename_R.c_str());
    FILE *f_L = fopen(file_L, "rb");
    FILE *f_R = fopen(file_R, "rb");

    if(f_L == nullptr || f_R == nullptr)
        return false;

    u_int8_t *cache_L = new u_int8_t[VLP16_BUFFER_SIZE];
    u_int8_t *cache_R = new u_int8_t[VLP16_BUFFER_SIZE];
    if(!fread(cache_L, sizeof(u_int8_t), VLP16_BUFFER_SIZE, f_L))
    {
        cerr<<"***Error: fread file "<<filename_L<<" failed!"<<endl;
        return false;
    }
    if(!fread(cache_R, sizeof(u_int8_t), VLP16_BUFFER_SIZE, f_R))
    {
        cout<<"***Error: fread file "<<filename_R<<" failed!"<<endl;
    }
    fclose(f_L);
    fclose(f_R);

    parse_lidar16_process(cache_L, cache_R);
    //calib_lidar16_data();

    delete[] cache_L;
    delete[] cache_R;
    return true;
}

void VELODYNE_PARSER::calib_lidar16_data()
{
	// 转换到32线雷达坐标系
    for(int beam=0; beam<VLP16_BEAM_NUM; beam++)
    {
        for(int cnt=0; cnt<VLP16_BEAM_POINTSIZE; cnt++)
        {
            mPoint3f *p = &lidar16_pointcloud_L[beam][cnt];
            mPoint3f tmp = lidar16_pointcloud_L[beam][cnt];
            if(p->valid)
            {
                p->x = tmp.x*para_table.lidar16_expara_L.R[0][0]+tmp.y*para_table.lidar16_expara_L.R[0][1]+tmp.z*para_table.lidar16_expara_L.R[0][2] + para_table.lidar16_expara_L.T[0];
                p->y = tmp.x*para_table.lidar16_expara_L.R[1][0]+tmp.y*para_table.lidar16_expara_L.R[1][1]+tmp.z*para_table.lidar16_expara_L.R[1][2] + para_table.lidar16_expara_L.T[1];
                p->z = tmp.x*para_table.lidar16_expara_L.R[2][0]+tmp.y*para_table.lidar16_expara_L.R[2][1]+tmp.z*para_table.lidar16_expara_L.R[2][2] + para_table.lidar16_expara_L.T[2];
            }

            p = &lidar16_pointcloud_R[beam][cnt];
            tmp = lidar16_pointcloud_R[beam][cnt];
            if(p->valid)
            {
                p->x = tmp.x*para_table.lidar16_expara_R.R[0][0]+tmp.y*para_table.lidar16_expara_R.R[0][1]+tmp.z*para_table.lidar16_expara_R.R[0][2] + para_table.lidar16_expara_R.T[0];
                p->y = tmp.x*para_table.lidar16_expara_R.R[1][0]+tmp.y*para_table.lidar16_expara_R.R[1][1]+tmp.z*para_table.lidar16_expara_R.R[1][2] + para_table.lidar16_expara_R.T[1];
                p->z = tmp.x*para_table.lidar16_expara_R.R[2][0]+tmp.y*para_table.lidar16_expara_R.R[2][1]+tmp.z*para_table.lidar16_expara_R.R[2][2] + para_table.lidar16_expara_R.T[2];
            }
        }
    }


/*	
	// 转换到车体坐标系
    for(int beam=0; beam<VLP16_BEAM_NUM; beam++)
    {
        for(int cnt=0; cnt<VLP16_BEAM_POINTSIZE; cnt++)
        {
            mPoint3f *p = &lidar16_pointcloud_L[beam][cnt];
            mPoint3f tmp = lidar16_pointcloud_L[beam][cnt];
            if(p->valid)
            {
                p->x = tmp.x*para_table.lidar32_expara.R[0][0]+tmp.y*para_table.lidar32_expara.R[0][1]+tmp.z*para_table.lidar32_expara.R[0][2];
                p->y = tmp.x*para_table.lidar32_expara.R[1][0]+tmp.y*para_table.lidar32_expara.R[1][1]+tmp.z*para_table.lidar32_expara.R[1][2];
                p->z = tmp.x*para_table.lidar32_expara.R[2][0]+tmp.y*para_table.lidar32_expara.R[2][1]+tmp.z*para_table.lidar32_expara.R[2][2];
            }

            p = &lidar16_pointcloud_R[beam][cnt];
            tmp = lidar16_pointcloud_R[beam][cnt];
            if(p->valid)
            {
                p->x = tmp.x*para_table.lidar32_expara.R[0][0]+tmp.y*para_table.lidar32_expara.R[0][1]+tmp.z*para_table.lidar32_expara.R[0][2];
                p->y = tmp.x*para_table.lidar32_expara.R[1][0]+tmp.y*para_table.lidar32_expara.R[1][1]+tmp.z*para_table.lidar32_expara.R[1][2];
                p->z = tmp.x*para_table.lidar32_expara.R[2][0]+tmp.y*para_table.lidar32_expara.R[2][1]+tmp.z*para_table.lidar32_expara.R[2][2];
            }
        }
    }
*/
}


// save lidar16 point cloud coordinates to text files
// each line corresponds to a point x, y, z, in cm.
void VELODYNE_PARSER::save_lidar16_txt(const std::string &filename_L, const std::string &filename_R)
{
    fstream fileout;
    fileout.open(filename_L, ios::out);
    if(!fileout)
        cerr<<"***Error: Can't open file \""<<filename_L<<"\""<<endl;
    for(int beam = 0; beam < VLP16_BEAM_NUM; beam++)
    {
        for(int angle = 0; angle < VLP16_BEAM_POINTSIZE; angle++)
        {
            if(lidar16_pointcloud_L[beam][angle].valid)
            {
                fileout << lidar16_pointcloud_L[beam][angle].x/100.0 << "\t\t"<<
                           lidar16_pointcloud_L[beam][angle].y/100.0 << "\t\t"<<
                           lidar16_pointcloud_L[beam][angle].z/100.0 << "\t\t"<<
						 							 lidar16_pointcloud_L[beam][angle].intensity<<endl;
            }
        }
    }
    fileout.close();

    fileout.open(filename_R, ios::out);
    if(!fileout)
        cerr<<"***Error: Can't open file \""<<filename_R<<"\""<<endl;
    for(int beam = 0; beam < VLP16_BEAM_NUM; beam++)
    {
        for(int angle = 0; angle < VLP16_BEAM_POINTSIZE; angle++)
        {
            if(lidar16_pointcloud_R[beam][angle].valid)
            {
                fileout << lidar16_pointcloud_R[beam][angle].x/100.0 << "\t\t"<<
                           lidar16_pointcloud_R[beam][angle].y/100.0 << "\t\t"<<
                           lidar16_pointcloud_R[beam][angle].z/100.0 << "\t\t"<<
						 							 lidar16_pointcloud_R[beam][angle].intensity<<endl;
            }
        }
    }
    fileout.close();
}


/*
 * Lidar32 part
*/
bool VELODYNE_PARSER::parse_lidar32_process(u_int8_t *cache)
{
    int packet_cnt = 0;
    u_int8_t *fp = cache + HDL32_PACKET_SIZE*packet_cnt;
    u_int8_t *fq = fp + 1;
    u_int16_t val = (*fp)*256 + *fq;

    float *cos_angH = para_table.lidar32_inpara.cos_angleH;
    float *sin_angH = para_table.lidar32_inpara.sin_angleH;
    float *cos_angV = para_table.lidar32_inpara.cos_angleV;
    float *sin_angV = para_table.lidar32_inpara.sin_angleV;
    float *angleV = para_table.lidar32_inpara.angleV;

	
    u_int16_t start_angleH = 1;	// 记录这一帧初始的水平向激光角度
    bool start_angleH_generated = false;
    while(val == 0xffee)
    {
        // parse packet, 1206 bytes
        for(int i=0; i<HDL32_NUM_SHOTS; i++)    // 12
        {
            u_int8_t *pAngL = fp + 100*i + 2;
            u_int8_t *pAngH = pAngL + 1;
            u_int16_t angleH = ((*pAngH)*256 + (*pAngL) + 9000)%36000; 
            u_int8_t *pL = fp + 100*i + 4;
            u_int8_t *pH = pL + 1;
            u_int8_t *pVal = pH + 1;
            int idx = packet_cnt*HDL32_NUM_SHOTS + i;
            if(!start_angleH_generated)
            {
              start_angleH = angleH;
              start_angleH_generated = true;
            }
            // time_ratio指的是当前扫扫描点相对于帧头的时间，用于后续畸变补偿
            float angleH_dis = float(angleH) - float(start_angleH);
            if(angleH_dis < 0.0)
              angleH_dis += 36000.0;
            float time_ratio = LIDAR_SPIN_CYCLE * angleH_dis / 36000.0;

            for(int j=0; j<HDL32_BEAM_NUM; j++) // 32
            {
                int distance = ((*pH)*256 + (*pL))*2;       // in mm
                if(distance >= HDL32_VALID_RADIUS)
                {
                    int beamInd = para_table.lidar32_inpara.beam_reverse_order[j];	//beamInd表示这个点实际上对应这按顺序排列的第beamInd个圈
                    lidar32_pointcloud[beamInd][idx].valid = true;
                    lidar32_pointcloud[beamInd][idx].distance = distance;
                    //lidar32_pointcloud[beamInd][idx].intensity = *pVal;
                    lidar32_pointcloud[beamInd][idx].intensity = (float)beamInd + time_ratio; // 改进的intensity整数部分存beam信息，
                                                                                              // 小数部分存当前点在周期中的时刻比值

                    lidar32_pointcloud[beamInd][idx].angleH = angleH;
                    lidar32_pointcloud[beamInd][idx].angleV = int16_t(angleV[j]*100);
                    lidar32_pointcloud[beamInd][idx].x = (float)distance/10.0 * cos_angV[j] * sin_angH[angleH];
                    lidar32_pointcloud[beamInd][idx].y = (float)distance/10.0 * cos_angV[j] * cos_angH[angleH];
                    lidar32_pointcloud[beamInd][idx].z = (float)distance/10.0 * sin_angV[j];
                    lidar32_pointNum++;
                }
                pL += 3;
                pH += 3;
                pVal += 3;
            }
        }

        // next packet
        packet_cnt++;
        if(packet_cnt > HDL32_PACKET_NUM - 1)       // in case of exceeding memory
            break;
        fp = cache + HDL32_PACKET_SIZE*packet_cnt;
        fq = fp + 1;
        val = (*fp)*256 + *fq;
    }
    return true;
}

bool VELODYNE_PARSER::parse_lidar32_data(const std::string filename)
{
    char file[256];
    strcpy(file, filename.c_str());
    FILE *fp = fopen(file, "rb");
    if(fp==NULL)
        return false;

    u_int8_t *cache = new u_int8_t[HDL32_BUFFER_SIZE];
    if(!fread(cache, sizeof(u_int8_t), HDL32_BUFFER_SIZE, fp))
    {
        cerr<<"***Error: fread file "<<filename<<" failed!"<<endl;
        return false;
    }
    fclose(fp);

    parse_lidar32_process(cache);
    //calib_lidar32_data();	//似乎不应该进行标定，因为loam里面使用了光线追踪的方法，要求点云坐标应该是以雷达为中心的
    delete[] cache;
    return true;
}

void VELODYNE_PARSER::calib_lidar32_data()
{
    for(int beam=0; beam<HDL32_BEAM_NUM; beam++)
    {
        for(int cnt=0; cnt<HDL32_BEAM_POINTSIZE; cnt++)
        {
            mPoint3f *p = &lidar32_pointcloud[beam][cnt];
            const mPoint3f tmp = lidar32_pointcloud[beam][cnt];
            if(p->valid)
            {
                p->x = tmp.x*para_table.lidar32_expara.R[0][0]+tmp.y*para_table.lidar32_expara.R[0][1]+tmp.z*para_table.lidar32_expara.R[0][2] + para_table.lidar32_expara.T[0];
                p->y = tmp.x*para_table.lidar32_expara.R[1][0]+tmp.y*para_table.lidar32_expara.R[1][1]+tmp.z*para_table.lidar32_expara.R[1][2] + para_table.lidar32_expara.T[1];
                p->z = tmp.x*para_table.lidar32_expara.R[2][0]+tmp.y*para_table.lidar32_expara.R[2][1]+tmp.z*para_table.lidar32_expara.R[2][2] + para_table.lidar32_expara.T[2];
            }
        }
    }
}

void VELODYNE_PARSER::save_lidar32_txt(const std::string &filename)
{
    fstream fileout;
    fileout.open(filename, ios::out);
    if(!fileout)
    {
        cerr<<"***Error: Can't open file \""<<filename<<"\""<<endl;
        return;
    }
    for(int beam = 0; beam < HDL32_BEAM_NUM; beam++)
    {
        for(int angle = 0; angle < HDL32_BEAM_POINTSIZE; angle++)
        {
            if(lidar32_pointcloud[beam][angle].valid)
            {
                fileout << lidar32_pointcloud[beam][angle].x/100.0 << "\t\t"<<
                           lidar32_pointcloud[beam][angle].y/100.0 << "\t\t"<<
                           lidar32_pointcloud[beam][angle].z/100.0 << "\t\t"<<
						   lidar32_pointcloud[beam][angle].intensity<<endl;
            }
        }
    }
    fileout.close();
}

int VELODYNE_PARSER::get_points_num_32()
{
  return lidar32_pointNum;
}


/*
 * Lidar64 part
*/
bool VELODYNE_PARSER::parse_lidar64_process(u_int8_t *cache)
{

  float *hCorrection = para_table.lidar64_inpara.hCorrection;
  float *distCorrection = para_table.lidar64_inpara.distCorrection;
  float *vOffsetCorrection = para_table.lidar64_inpara.vOffsetCorrection;
  float *hOffsetCorrection = para_table.lidar64_inpara.hOffsetCorrection;

  float *cos_angH = para_table.lidar64_inpara.cos_angleH;
  float *sin_angH = para_table.lidar64_inpara.sin_angleH;
  float *cos_angV = para_table.lidar64_inpara.cos_angleV;
  float *sin_angV = para_table.lidar64_inpara.sin_angleV;
  float *angleV = para_table.lidar64_inpara.angleV;

  bool start_angle_determined = false;
  float start_angle = 0.0;
  hdl_packet_t * pPacket = new hdl_packet_t;
  int shotCnt = 0;
  // every packet, 1206 bytes
  for(int packetInd = 0; packetInd < HDL64_PACKET_NUM; packetInd++)  // 350
  {
    memcpy(pPacket, cache+sizeof(hdl_packet_t)*packetInd, sizeof(hdl_packet_t));

    // every shot, 100 bytes
    for(int shotInd = 0; shotInd < HDL64_NUM_SHOTS; shotInd++)      // 12
    {
      shotCnt++;
      hdl_shot_t &shot = pPacket->shots[shotInd];
      const u_int16_t flag = shot.lower_upper;
      if(flag == 0x0000)
        break;

      u_int8_t *pL = (u_int8_t *)&(shot.rotational_angle);
      u_int8_t *pH = pL+1;
      u_int16_t rotAngle = 256 * (*pH) + (*pL);
      if(!start_angle_determined)
      {
        start_angle = (float)rotAngle/100.0 - hCorrection[0];
        start_angle_determined = true;
      }

      // every laser, 3 bytes
      for(int laserInd = 0; laserInd < HDL64_NUM_LASERS; laserInd++)// 32
      {
        int k = 0;
        if(flag == 0xeeff/*HDL64_SHOT_UPPER_FLAG*/)
          k = laserInd;
        else if(flag == HDL64_SHOT_LOWER_FLAG)
          k = laserInd + HDL64_NUM_LASERS;

        hdl_laser_t &laser = shot.lasers[laserInd];
        pL = (u_int8_t *)&(laser.distance);
        pH = pL+1;
        u_int16_t distance = 256*(*pH) + (*pL);
        if(distance == 0x0000)
          continue;
        u_int8_t intensity = laser.intensity;

        float realDistance = (float)distance * 0.2 + distCorrection[k];
        if(realDistance < (float)HDL64_VALID_RADIUS)
          continue;

        float realAngle = (float)rotAngle/100.0 - hCorrection[k];
        float timeRatio = LIDAR_SPIN_CYCLE * (realAngle - start_angle + 360.0 * (realAngle >= start_angle ? 0 : 1)) / 360.0;
        float realRotRad = realAngle/180.0*M_PI;
        float vOffsetCoor = vOffsetCorrection[k];
        float hOffsetCoor = hOffsetCorrection[k];
        float xyDistance = realDistance * cos_angV[k] - vOffsetCoor * sin_angV[k];
        int beamInd = para_table.lidar64_inpara.beam_reverse_order[k];
        int idx = shotCnt/2;
        lidar64_pointcloud[beamInd][idx].valid = true;
        lidar64_pointcloud[beamInd][idx].distance = distance;
        lidar64_pointcloud[beamInd][idx].intensity = (float)beamInd + timeRatio;
        lidar64_pointcloud[beamInd][idx].angleH = rotAngle;
        lidar64_pointcloud[beamInd][idx].angleV = int16_t(angleV[k]*100);
        lidar64_pointcloud[beamInd][idx].x = xyDistance * sin(realRotRad) - hOffsetCoor * cos(realRotRad);
        lidar64_pointcloud[beamInd][idx].y = xyDistance * cos(realRotRad) + hOffsetCoor * sin(realRotRad);
        lidar64_pointcloud[beamInd][idx].z = realDistance * sin_angV[k] + vOffsetCoor * cos_angV[k];
        lidar64_pointNum++;
      }

    }
  }

  cout<<"lidar64_pointNum: "<<lidar64_pointNum<<endl;
  delete pPacket;
  return true;
}

bool VELODYNE_PARSER::parse_lidar64_data(const std::string filename)
{
    char file[256];
    strcpy(file, filename.c_str());
    FILE *fp = fopen(file, "rb");
    if(fp==NULL)
    {
        cerr << "***Error: Can not open file: " << filename << endl;
        return false;
    }

    u_int8_t *cache = new u_int8_t[HDL64_BUFFER_SIZE];
    if(!fread(cache, sizeof(u_int8_t), HDL64_BUFFER_SIZE, fp))
    {
        cerr<<"***Error: fread file "<<filename<<" failed!"<<endl;
        return false;
    }
    fclose(fp);

    parse_lidar64_process(cache);
    //calib_lidar64_data();	//似乎不应该进行标定，因为loam里面使用了光线追踪的方法，要求点云坐标应该是以雷达为中心的
    delete[] cache;
    return true;
}

void VELODYNE_PARSER::calib_lidar64_data()
{
    for(int beam=0; beam<HDL64_BEAM_NUM; beam++)
    {
        for(int cnt=0; cnt<HDL64_BEAM_POINTSIZE; cnt++)
        {
            mPoint3f *p = &lidar64_pointcloud[beam][cnt];
            const mPoint3f tmp = lidar64_pointcloud[beam][cnt];
            if(p->valid)
            {
                p->x = tmp.x*para_table.lidar64_expara.R[0][0]+tmp.y*para_table.lidar64_expara.R[0][1]+tmp.z*para_table.lidar64_expara.R[0][2] + para_table.lidar64_expara.T[0];
                p->y = tmp.x*para_table.lidar64_expara.R[1][0]+tmp.y*para_table.lidar64_expara.R[1][1]+tmp.z*para_table.lidar64_expara.R[1][2] + para_table.lidar64_expara.T[1];
                p->z = tmp.x*para_table.lidar64_expara.R[2][0]+tmp.y*para_table.lidar64_expara.R[2][1]+tmp.z*para_table.lidar64_expara.R[2][2] + para_table.lidar64_expara.T[2];
            }
        }
    }

}

void VELODYNE_PARSER::save_lidar64_txt(const std::string &filename)
{
    fstream fileout;
    fileout.open(filename, ios::out);
    if(!fileout)
    {
        cerr<<"***Error: Can't open file \""<<filename<<"\""<<endl;
        return;
    }
    for(int beam = 0; beam < HDL64_BEAM_NUM; beam++)
    {
        for(int angle = 0; angle < HDL64_BEAM_POINTSIZE; angle++)
        {
            if(lidar64_pointcloud[beam][angle].valid)
            {
                fileout << lidar64_pointcloud[beam][angle].x << "\t\t"<<
                           lidar64_pointcloud[beam][angle].y << "\t\t"<<
                           lidar64_pointcloud[beam][angle].z << "\t\t"<<
                           lidar64_pointcloud[beam][angle].intensity<<endl;
            }
        }
    }
    fileout.close();
}

int VELODYNE_PARSER::get_points_num_64()
{
  return lidar64_pointNum;
}
