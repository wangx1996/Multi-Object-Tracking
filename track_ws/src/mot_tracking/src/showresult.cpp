#include <iostream>
#include <vector>
#include <stdlib.h>
#include <string>
#include <chrono>
#include <fstream>
#include <sstream>
#include <vector>
#include <ctime>
#include <time.h> 
#include <unordered_map>
#include <dirent.h>
#include <sys/types.h>    
#include <sys/stat.h> 

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <boost/tokenizer.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

#include "lonlat2utm.h"

using namespace std;
typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

template <class Type>  
Type stringToNum(const string& str)  
{  
    istringstream iss(str);  
    Type num;  
    iss >> num;  
    return num;      
}  

template<typename T> string toString(const T& t) {
	ostringstream oss;
	oss << t;
	return oss.str();
}


int fileNameFilter(const struct dirent *cur) {
	std::string str(cur->d_name);
	if (str.find(".bin") != std::string::npos
			|| str.find(".pcd") != std::string::npos
			|| str.find(".png") != std::string::npos
			|| str.find(".jpg") != std::string::npos
			|| str.find(".txt") != std::string::npos) {
		return 1;
	}
	return 0;
}


bool get_all_files(const std::string& dir_in,
		std::vector<std::string>& files) {

	if (dir_in.empty()) {
		return false;
	}
	struct stat s;
	stat(dir_in.c_str(), &s);
	if (!S_ISDIR(s.st_mode)) {
		return false;
	}
	DIR* open_dir = opendir(dir_in.c_str());
	if (NULL == open_dir) {
		std::exit(EXIT_FAILURE);
	}
	dirent* p = nullptr;
	while ((p = readdir(open_dir)) != nullptr) {
		struct stat st;
		if (p->d_name[0] != '.') {
			//因为是使用devC++ 获取windows下的文件，所以使用了 "\" ,linux下要换成"/"
			//cout<<std::string(p->d_name)<<endl;
			std::string name = dir_in + std::string("/")
					+ std::string(p->d_name);
			stat(name.c_str(), &st);
			if (S_ISDIR(st.st_mode)) {
				get_all_files(name, files);
			} else if (S_ISREG(st.st_mode)) {
				boost::char_separator<char> sepp { "." };
				tokenizer tokn(std::string(p->d_name), sepp);
				vector<string> filename_sep(tokn.begin(), tokn.end());
				string type_ = "." + filename_sep[1];
				break;
			}
		}
	}

	struct dirent **namelist;
	int n = scandir(dir_in.c_str(), &namelist, fileNameFilter, alphasort);
	if (n < 0) {
		return false;
	}
	for (int i = 0; i < n; ++i) {
		std::string filePath(namelist[i]->d_name);
		files.push_back(filePath);
		free(namelist[i]);
	};
	free(namelist);
	closedir(open_dir);
	return true;
}

bool LoadDetectionPath(std::vector<std::string>& trackfile_name,std::vector<std::string>& vpose_name, std::vector<std::string>& gps_name, string& path){
	string tracker_file_path = path + "/results";
	cout<<tracker_file_path<<endl;
	if(!get_all_files(tracker_file_path, trackfile_name))
		return false;

	string pose_file_path = path + "/vpose";
	cout<<pose_file_path<<endl;
	if(!get_all_files(pose_file_path, vpose_name))
		return false;


	string gps_file_path = path + "/gps";
	cout<<gps_file_path<<endl;
	if(!get_all_files(gps_file_path, gps_name))
		return false;

	cout<<vpose_name.size()<<" "<<trackfile_name.size()<<" "<< gps_name.size()<<endl;

	if(vpose_name.size() != trackfile_name.size()){
		return false;
	}

	if(vpose_name.size() != gps_name.size()){
		return false;
	}

	return true;
}


int main(int argc, char** argv){

	string path = argv[1];

	string file = argv[2];
	
	std::vector<std::string> tracker;
	std::vector<std::string> pose;
	std::vector<std::string> gps;

	if(!LoadDetectionPath(tracker, pose, gps, path)){
		std::abort();
	}

	vector<vector<double>> gpsfile;
	unordered_map<int, vector<vector<float>> > trackdets;
	vector<vector<float>> poses;
	boost::char_separator<char> sep { " " };
	int frame = 0;
	while(frame < tracker.size()){
		string gps_filename_path = path + "/gps/" + gps[frame];
		string tracker_filename_path = path + "/results/" + tracker[frame];
		string pose_filename_path = path + "/vpose/" + pose[frame];

		cout<<tracker_filename_path<<endl;
		std::ifstream gpsf(gps_filename_path);
		if(gpsf){
			boost::char_separator<char> sep_line { "\n" };
			std::stringstream buffer;
			buffer << gpsf.rdbuf();
			std::string contents(buffer.str());
			tokenizer tok_line(contents, sep_line);
			std::vector<std::string> lines(tok_line.begin(), tok_line.end());
			vector<string> gpsd = lines;

			tokenizer tokn(gpsd[0], sep);
			vector<string> temp_sep(tokn.begin(), tokn.end());

			double longitude = stringToNum<double>(temp_sep[0]);
			double latitude = stringToNum<double>(temp_sep[1]);
			double heading = stringToNum<double>(temp_sep[2])*M_PI/180;// - 90 * M_PI/180;
			//std::cout<<fixed<<setprecision(8)<<longitude<<" "<<latitude<<" "<<heading<<std::endl;
			vector<double> gpss(3,0);
			gpss[0] = longitude;
			gpss[1] = latitude;
			gpss[2] = heading;
			gpsfile.push_back(gpss);
		}

		std::ifstream trackf(tracker_filename_path);
		if(trackf){
			boost::char_separator<char> sep_line { "\n" };
			std::stringstream buffer;
			buffer << trackf.rdbuf();
			std::string contents(buffer.str());
			tokenizer tok_line(contents, sep_line);
			std::vector<std::string> lines(tok_line.begin(), tok_line.end());
			vector<string> trackr = lines;
			int size = trackr.size();
			vector<vector<float>> trackdetes;
			for(int i=0; i<size; ++i){
				vector<float> trackdet(10,0);//calss, id, x,y,velo, mx, my, yaw, w,l
				tokenizer tokn(trackr[i], sep);
				vector<string> temp_sep(tokn.begin(), tokn.end());
				trackdet[0] = stringToNum<float>(temp_sep[0]);
				trackdet[1] = stringToNum<float>(temp_sep[1]);
				trackdet[2] = stringToNum<float>(temp_sep[2]);
				trackdet[3] = stringToNum<float>(temp_sep[3]);
				trackdet[4] = stringToNum<float>(temp_sep[4]);
				trackdet[5] = stringToNum<float>(temp_sep[5]);
				trackdet[6] = stringToNum<float>(temp_sep[6]);
				trackdet[7] = stringToNum<float>(temp_sep[7]);
				trackdet[8] = stringToNum<float>(temp_sep[8]);
				trackdet[9] = stringToNum<float>(temp_sep[9]);
				cout<<trackdet[0]<< " "<<trackdet[1]<<" "<<trackdet[2]<<" "<<trackdet[3]<<endl;
				trackdetes.push_back(trackdet);
			}
			trackdets[frame] = trackdetes;
		}


		std::ifstream posef(pose_filename_path);
		if(posef){
			boost::char_separator<char> sep_line { "\n" };
			std::stringstream buffer;
			buffer << posef.rdbuf();
			std::string contents(buffer.str());
			tokenizer tok_line(contents, sep_line);
			std::vector<std::string> lines(tok_line.begin(), tok_line.end());
			vector<string> poset = lines;
			int size = poset.size();
			for(int i=0; i<size; ++i){
				vector<float> posedet(9,0);//v1x v1y v2x v2y angle minh maxh minw maxw
				tokenizer tokn(poset[i], sep);
				vector<string> temp_sep(tokn.begin(), tokn.end());
				posedet[0] = stringToNum<float>(temp_sep[0]);
				posedet[1] = stringToNum<float>(temp_sep[1]);
				posedet[2] = stringToNum<float>(temp_sep[2]);
				posedet[3] = stringToNum<float>(temp_sep[3]);
				posedet[4] = stringToNum<float>(temp_sep[4]);
				posedet[5] = stringToNum<float>(temp_sep[5]);
				posedet[6] = stringToNum<float>(temp_sep[6]);
				posedet[7] = stringToNum<float>(temp_sep[7]);
				posedet[8] = stringToNum<float>(temp_sep[8]);
				poses.push_back(posedet);
			}
		}
		frame++;
	}

	int fram = stringToNum<float>(file);

	double UTME,UTMN;
	Eigen::Matrix3d rotZorigion;
	Eigen::Isometry3d porigion;
 
	LonLat2UTM(gpsfile[fram][0],gpsfile[fram][1], UTME, UTMN);
	Eigen::AngleAxisd roto(gpsfile[fram][2] ,Eigen::Vector3d::UnitZ());
	rotZorigion = roto.toRotationMatrix();
	porigion = rotZorigion;
	porigion.translation() = Eigen::Vector3d(UTME, UTMN, 0);


	double UTMEs,UTMNs;
	Eigen::Matrix3d rotZs;
	Eigen::Isometry3d ps;
 
	LonLat2UTM(gpsfile[0][0],gpsfile[0][1], UTMEs, UTMNs);
	Eigen::AngleAxisd rots(gpsfile[0][2] ,Eigen::Vector3d::UnitZ());
	rotZs = rots.toRotationMatrix();
	ps = rotZs;
	ps.translation() = Eigen::Vector3d(UTMEs, UTMNs, 0);

	Eigen::Isometry3d tdet2now = porigion.inverse()*ps;

	std::stringstream ss;
	ss << setw(6) << setfill('0') << stringToNum<float>(file) << ".png";
	string imgpath = path+ "/grid/" +ss.str();
	cout<<imgpath<<endl;
	cv::Mat image = cv::imread(imgpath);;

	float dx = 20 -poses[fram][7]*0.2;
	float dy = 20 -poses[fram][5]*0.2;

	vector<cv::Point> tr;
	vector<cv::Point> trp;
	vector<cv::Point> v1p;
	vector<cv::Point> v2p;
	vector<float> v2yaw;
	vector<float> v1yaw;
	vector<float> tryaw;
	vector<float> trpyaw;

	for(int i=0; i<fram; ++i){


		double UTMEn,UTMNn;
		Eigen::Matrix3d rotZn; 
		LonLat2UTM(gpsfile[i][0],gpsfile[i][1], UTMEn, UTMNn);
		Eigen::AngleAxisd roton(gpsfile[i][2] ,Eigen::Vector3d::UnitZ());
		rotZn = roton.toRotationMatrix();
		Eigen::Isometry3d pn;
		pn = rotZn;
		pn.translation() = Eigen::Vector3d(UTMEn, UTMNn, 0);
		
		Eigen::Isometry3d t12 = porigion.inverse()*pn;

		Eigen::Vector3d v2(poses[i][2],poses[i][3],0);
		Eigen::Vector3d v2transform = t12*v2;

		float cx = (v2transform(0)+dx)/0.2;
		float cy = (v2transform(1)+dy)/0.2;
		//cout<<imgpath<<endl;
		cout<<cx<<" "<<cy<<" "<<image.cols<<" "<<image.rows<<endl;

		if(cx >=0 && cx < image.cols && cy >=0 && cy < image.rows){
			v2p.push_back(cv::Point((v2transform(0)+dx)/0.2,  (v2transform(1)+dy)/0.2));
			//cv::circle(image, cv::Point((v2transform(0)+dx)/0.2,  (v2transform(1)+dy)/0.2), 2, cv::Scalar(0,0,255),1);
			float yaw = poses[i][4] + gpsfile[i][2] - gpsfile[fram][2];
			v2yaw.push_back(yaw);
		}
		Eigen::Vector3d v1(poses[i][0],poses[i][1],0);
		Eigen::Vector3d v1transform = t12*v1;
		float v1x = (v1transform(0)+dx)/0.2;
		float v1y = (v1transform(1)+dy)/0.2;
		if(v1x >=0 && v1x < image.cols && v1y >=0 && v1y < image.rows){
			v1p.push_back(cv::Point((v1transform(0)+dx)/0.2,  (v1transform(1)+dy)/0.2));
			//cv::circle(image, cv::Point((v1transform(0)+dx)/0.2,  (v1transform(1)+dy)/0.2), 2, cv::Scalar(0,255,255),1);
			float yaw = gpsfile[i][2] - gpsfile[fram][2];
			v1yaw.push_back(yaw);
		}

		int id = 1;
		cout<<trackdets[i].size()<<endl;
		for(int j=0; j<trackdets[i].size(); ++j){
			if(trackdets[i][j][1] == id && trackdets[i][j][0] == 1){
				Eigen::Vector3d track(trackdets[i][j][2],trackdets[i][j][3],0);
				cout<<"###########start############ "<<i<<endl;
				cout<<track(0)<<" "<<track(1)<<endl;
				Eigen::Vector3d tracktransform = tdet2now*track;
				cout<<tracktransform(0)<<" "<<tracktransform(1)<<endl;
				cout<<"###########end############"<<endl;
				float tr1x = (tracktransform(0)+dx)/0.2;
				float tr1y = (tracktransform(1)+dy)/0.2;
				if(tr1x >=0 && tr1x < image.cols && tr1y >=0 && tr1y < image.rows){
					tr.push_back(cv::Point((tracktransform(0)+dx)/0.2,  (tracktransform(1)+dy)/0.2));
					//cv::circle(image, cv::Point((tracktransform(0)+dx)/0.2,  (tracktransform(1)+dy)/0.2), 2, cv::Scalar(255,0,0),1);
					float yaw = trackdets[i][j][7]- gpsfile[fram][2];
					tryaw.push_back(yaw);
				}
			}
		}


		id = 0;

		for(int j=0; j<trackdets[i].size(); ++j){
			if(trackdets[i][j][1] == id && trackdets[i][j][0] == 2){
				Eigen::Vector3d track(trackdets[i][j][2],trackdets[i][j][3],0);
				cout<<"###########start############ "<<i<<endl;
				cout<<track(0)<<" "<<track(1)<<endl;
				Eigen::Vector3d tracktransform = tdet2now*track;
				cout<<tracktransform(0)<<" "<<tracktransform(1)<<endl;
				cout<<"###########end############"<<endl;
				float tr1x = (tracktransform(0)+dx)/0.2;
				float tr1y = (tracktransform(1)+dy)/0.2;
				if(tr1x >=0 && tr1x < image.cols && tr1y >=0 && tr1y < image.rows){
					trp.push_back(cv::Point((tracktransform(0)+dx)/0.2,  (tracktransform(1)+dy)/0.2));
					//cv::circle(image, cv::Point((tracktransform(0)+dx)/0.2,  (tracktransform(1)+dy)/0.2), 2, cv::Scalar(255,0,0),1);
					float yaw = trackdets[i][j][7]- gpsfile[fram][2];
					trpyaw.push_back(yaw);
				}
			}
		}
		
		
	}

	for(int i=0; i<v2p.size()-1; ++i){
		cv::line(image, v2p[i], v2p[i+1],  cv::Scalar(0,255,0),2);
		if((i%10)==0){
			//cout<<"########"<<i<<endl;
			cv::RotatedRect rect = cv::RotatedRect(v2p[i], cv::Size2f(2.2/0.2, 4/0.2), v2yaw[i]);
			cv::Point2f vertices[4];
    			rect.points(vertices);				
    			for (int i = 0; i < 4; i++){
        				cv::line(image, vertices[i], vertices[(i+1)%4],  cv::Scalar(0,255,0), 2);
			}
		}
	}


	for(int i=0; i<v1p.size()-1; ++i){
		cv::line(image, v1p[i], v1p[i+1],  cv::Scalar(255,0,0),2);
		if((i%40)==0){
			//cout<<"########"<<i<<endl;
			cv::RotatedRect rect = cv::RotatedRect(v1p[i], cv::Size2f(2.2/0.2, 4/0.2), v1yaw[i]);
			cv::Point2f vertices[4];
    			rect.points(vertices);				
    			for (int i = 0; i < 4; i++){
        				cv::line(image, vertices[i], vertices[(i+1)%4],  cv::Scalar(255,0,0), 2);
			}
		}
	}


	if(tr.size()>0){
		for(int i=0; i<tr.size()-1; ++i){
			cv::line(image, tr[i], tr[i+1],  cv::Scalar(0,0,255),2);
			if((i%20)==0){
				cout<<"########"<<i<<" "<<tryaw[i]<<endl;
				cv::RotatedRect rect = cv::RotatedRect(tr[i], cv::Size2f(2.2/0.2, 4/0.2), tryaw[i]+20);
				cv::Point2f vertices[4];
    				rect.points(vertices);				
    				for (int i = 0; i < 4; i++){
        				cv::line(image, vertices[i], vertices[(i+1)%4],  cv::Scalar(0,0,255), 2);
				}
			}
		}
	}

	if(trp.size()>0){
		for(int i=0; i<trp.size()-1; ++i){
			cv::line(image, trp[i], trp[i+1],  cv::Scalar(255,191,0),2);
			if((i%20)==0){
				cout<<"########"<<i<<" "<<trpyaw[i]<<endl;
				cv::RotatedRect rect = cv::RotatedRect(trp[i], cv::Size2f(0.6/0.2, 1.2/0.2), trpyaw[i]);
				cv::Point2f vertices[4];
    				rect.points(vertices);				
    				for (int i = 0; i < 4; i++){
        				cv::line(image, vertices[i], vertices[(i+1)%4],  cv::Scalar(255,191,0), 2);
				}
			}
		}
	}

	cv::resize(image,image,cv::Size(image.cols*2,image.rows*2));
	cv::imshow("test", image);
	cv::waitKey(0);
	

	return 0;
}


