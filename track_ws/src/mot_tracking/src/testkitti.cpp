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
#include <sys/time.h>

//Eigen
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

//Boost
#include <boost/tokenizer.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include "lonlat2utm.h"

#include"readparam.h"
#include"tracker.h"

//Ros
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h> 
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>


using namespace std;
typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
static string folder_lidar = "results/";
static string format_lidar = "%|06|.txt";

static int64_t gtm() {
	struct timeval tm;
	gettimeofday(&tm, 0);
	// return ms
	int64_t re = (((int64_t) tm.tv_sec) * 1000 * 1000 + tm.tv_usec);
	return re;
}


struct pose{
	double x;
	double y;
	double heading;
};

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

bool Load_Sensor_Data_Path(std::vector<std::string>& lidarfile_name, std::vector<std::string>& imagefile_name, string& path){
	string lidar_file_path = path + "/velodyne";
	cout<<lidar_file_path<<endl;
	if(!get_all_files(lidar_file_path, lidarfile_name))
		return false;
	string image_file_path = path + "/image_2";
	cout<<image_file_path<<endl;
	if(!get_all_files(image_file_path, imagefile_name))
		return false;
	if(lidarfile_name.size()!= imagefile_name.size())
		return false;
	return true;
}


cv::Point cloud2camera(Eigen::Vector3d input){
	Eigen::Matrix4d RT_velo_to_cam;
	Eigen::Matrix4d R_rect;
	Eigen::MatrixXd project_matrix(3,4);
    	RT_velo_to_cam<<7.49916597e-03, -9.99971248e-01, -8.65110297e-04, -6.71807577e-03,
    			1.18652889e-02, 9.54520517e-04, -9.99910318e-01, -7.33152811e-02,
    			9.99882833e-01, 7.49141178e-03, 1.18719929e-02, -2.78557062e-01,
			0, 0, 0, 1;
    	R_rect<<0.99992475, 0.00975976, -0.00734152, 0,
    		-0.0097913, 0.99994262, -0.00430371, 0,
    		0.00729911, 0.0043753, 0.99996319, 0,
   		0, 0, 0, 1;
    	project_matrix<<7.215377e+02,0.000000e+00,6.095593e+02,4.485728e+01,
                    0.000000e+00,7.215377e+02,1.728540e+02,2.163791e-01,
                    0.000000e+00,0.000000e+00,1.000000e+00,2.745884e-03;
    	Eigen::MatrixXd transform_matrix_ = project_matrix*R_rect*RT_velo_to_cam;

	Eigen::Vector4d point;
	point<<input(0), input(1), input(2), 1;
	Eigen::Vector3d pimage = transform_matrix_* point;
	cv::Point p2d = cv::Point(pimage(0)/pimage(2),pimage(1)/pimage(2));
	return p2d;
}

Eigen::Vector3d camera2cloud(Eigen::Vector3d input){
	Eigen::Matrix4d RT_velo_to_cam;
	Eigen::Matrix4d R_rect;
    	RT_velo_to_cam<<7.49916597e-03, -9.99971248e-01, -8.65110297e-04, -6.71807577e-03,
    			1.18652889e-02, 9.54520517e-04, -9.99910318e-01, -7.33152811e-02,
    			9.99882833e-01, 7.49141178e-03, 1.18719929e-02, -2.78557062e-01,
			0, 0, 0, 1;
    	R_rect<<0.99992475, 0.00975976, -0.00734152, 0,
    		-0.0097913, 0.99994262, -0.00430371, 0,
    		0.00729911, 0.0043753, 0.99996319, 0,
   		0, 0, 0, 1;

	Eigen::Vector4d point;
	point<<input(0), input(1), input(2), 1;
	Eigen::Vector4d pcloud = RT_velo_to_cam.inverse()*R_rect.inverse()* point;
	Eigen::Vector3d result;
	result<<pcloud(0),pcloud(1),pcloud(2);
	return result;
}

void draw3dbox(Detect &det, cv::Mat& image, vector<int>& color){
	float h = det.box[0];	
	float w = det.box[1];
	float l = det.box[2];
        float x = det.position[0];
	float y = det.position[1];
	float z = det.z;
	float yaw = -det.yaw - 90 * M_PI/180;
	double boxroation[9] = {cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1 };

	Eigen::MatrixXd BoxRotation = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >(boxroation);
	double xAxisP[8] = {l/2,l/2,-l/2,-l/2,l/2,l/2,-l/2,-l/2 }; 
	double yAxisP[8] = {w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2}; 
	double zAxisP[8] = {h, h, h, h, 0, 0, 0, 0}; 
	vector<cv::Point> imagepoint;
	Eigen::Vector3d translation(x,y,z);

	for (int i = 0; i < 8; i++) {
		Eigen::Vector3d point_3d(xAxisP[i], yAxisP[i], zAxisP[i]);
		Eigen::Vector3d rotationPoint_3d = BoxRotation * point_3d + translation;
		cv::Point imgpoint = cloud2camera(rotationPoint_3d);
		imagepoint.push_back(imgpoint);
	}

	int r = color[0];
	int g = color[1];
	int b = color[2];

	cv::line(image, imagepoint[0], imagepoint[1],cv::Scalar(226, 43, 138), 2, CV_AA);
	cv::line(image, imagepoint[1], imagepoint[2],cv::Scalar(r, g, b), 2, CV_AA);
	cv::line(image, imagepoint[2], imagepoint[3],cv::Scalar(r, g, b), 2, CV_AA);
	cv::line(image, imagepoint[3], imagepoint[0],cv::Scalar(r, g, b), 2, CV_AA);

	cv::line(image, imagepoint[4], imagepoint[5],cv::Scalar(226, 43, 138), 2, CV_AA);
	cv::line(image, imagepoint[5], imagepoint[6],cv::Scalar(r, g, b), 2, CV_AA);
	cv::line(image, imagepoint[6], imagepoint[7],cv::Scalar(r, g, b), 2, CV_AA);
	cv::line(image, imagepoint[7], imagepoint[4],cv::Scalar(r, g, b), 2, CV_AA);

	cv::line(image, imagepoint[0], imagepoint[4],cv::Scalar(226, 43, 138), 2, CV_AA);
	cv::line(image, imagepoint[1], imagepoint[5],cv::Scalar(226, 43, 138), 2, CV_AA);
	cv::line(image, imagepoint[2], imagepoint[6],cv::Scalar(r, g, b), 2, CV_AA);
	cv::line(image, imagepoint[3], imagepoint[7],cv::Scalar(r, g, b), 2, CV_AA);

	cv::line(image, imagepoint[0], imagepoint[5],cv::Scalar(226, 43, 138), 2, CV_AA);
	cv::line(image, imagepoint[1], imagepoint[4],cv::Scalar(226, 43, 138), 2, CV_AA);

	imagepoint.clear();

}


bool LoadPointcloud(pcl::PointCloud<pcl::PointXYZI>& cloud_IN, string path){
	string lidar_filename_path = path;
	ifstream inputfile;
	inputfile.open(lidar_filename_path, ios::binary);
	if (!inputfile) {
		cerr << "ERROR: Cannot open file " << lidar_filename_path
					<< "! Aborting..." << endl;
		return false;
	}

	inputfile.seekg(0, ios::beg);
	for (int i = 0; inputfile.good() && !inputfile.eof(); i++) {
		pcl::PointXYZI data;
		inputfile.read(reinterpret_cast<char*>(&data), sizeof(data));
		pcl::PointXYZI p;
		p.x = data.x;
		p.y = data.y;
		p.z = data.z;
		p.intensity = data.intensity;
		//cout<<p.x<<" "<<p.y<<" "<<p.z<<" "<<endl;
		cloud_IN.points.push_back(p);
	}
	return true;
}

int main(int argc, char** argv){

 	ros::init(argc, argv, "vision_node");

  	ros::NodeHandle nh;


  	image_transport::ImageTransport it(nh);
  	image_transport::Publisher pubimage = it.advertise("/mot_tracking/image", 1);
 	ros::Publisher publidar = nh.advertise <sensor_msgs::PointCloud2> ("/mot_tracking/pointcloud", 1);
  	ros::Publisher pubmarker = nh.advertise<visualization_msgs::MarkerArray>("/mot_tracking/box", 1 );
  	ros::Publisher pubtextmarker = nh.advertise<visualization_msgs::MarkerArray>("/mot_tracking/id", 1 );

	//data path
	string datapath;
	ros::param::get("/tracking_node/datapath", datapath);

	//data number
	string file;
	ros::param::get("/tracking_node/file", file);

	//tracking class
	string trackclass;
	ros::param::get("/tracking_node/trackclass", trackclass);

	ros::Rate r(10);

	Param param;
	Tracker tracker(param);
	std::string path = datapath+file;
	cout<<path<<endl;

	std::vector<std::string>  lidarname;
	std::vector<std::string>  imagename;
    	float time = 0.1;

    	boost::filesystem::path result_dir_path = boost::filesystem::path(path)
                       / folder_lidar;
  	if(!boost::filesystem::exists(result_dir_path)) {
    		boost::filesystem::create_directories(result_dir_path);
 	}


	if(!Load_Sensor_Data_Path(lidarname, imagename, path)){
		cout<<"Detecion file wrong!"<<endl;
		std::abort();
	}

	std::unordered_map<string,int> classname2id;
	classname2id["Car"] = 1;
	classname2id["Pedestrian"] = 2;	
	classname2id["Cyclist"] = 3;	

	//read gps data get the longitude latitude
	std::string gpspath = path + "/oxts/"+file+".txt";
	cout<<gpspath<<endl;
	std::ifstream gps(gpspath);
	std::vector<std::string> gpsdata;
	if (gps) {
		boost::char_separator<char> sep_line { "\n" };
		std::stringstream buffer;
		buffer << gps.rdbuf();
		std::string contents(buffer.str());
		tokenizer tok_line(contents, sep_line);
		std::vector<std::string> lines(tok_line.begin(), tok_line.end());
		gpsdata = lines;
		int size = gpsdata.size();

		cout<<size<<" "<< lidarname.size()<<endl;
		if(size != lidarname.size()){
			cout<<"file number wrong!"<<endl;
			std::abort();
		}
	}

	boost::char_separator<char> sep { " " };
	int maxframe = 0;

	// read the label file
	std::string labelpath = path + "/lable_02/"+file+".txt";
	cout<<labelpath<<endl;
	std::ifstream label(labelpath);
	std::vector<std::string> labeldata;
	if (label) {
		boost::char_separator<char> sep_line { "\n" };
		std::stringstream buffer;
		buffer << label.rdbuf();
		std::string contents(buffer.str());
		tokenizer tok_line(contents, sep_line);
		std::vector<std::string> lines(tok_line.begin(), tok_line.end());
		labeldata = lines;
		int size = labeldata.size();
		
		tokenizer tokn(labeldata[size-1], sep);
		vector<string> temp_sep(tokn.begin(), tokn.end());
		maxframe = stringToNum<int>(temp_sep[0]);
		cout<<maxframe<<" "<< lidarname.size()<<endl;
		if((maxframe+1) != lidarname.size()){
			cout<<"file number wrong!"<<endl;
			std::abort();
		}
	}


	unordered_map<int, vector<Detect>> Inputdets;

	int max_truncation = 0;
	int max_occlusion = 2;

	for(int i=0; i<labeldata.size(); ++i){
		tokenizer tokn(labeldata[i], sep);
		vector<string> temp_sep(tokn.begin(), tokn.end());
		int fra        =  stringToNum<int>(temp_sep[0]);    //frame
		string type    = temp_sep[2];                       //class
		int truncation = stringToNum<int>(temp_sep[3]);     //truncation
		int occlusion  = stringToNum<int>(temp_sep[4]);     //occlusion
		float x1       = stringToNum<float>(temp_sep[6]);   //left[pixel]
		float y1       = stringToNum<float>(temp_sep[7]);   //top[pixel]
		float x2       = stringToNum<float>(temp_sep[8]);   //right[pixel]
		float y2       = stringToNum<float>(temp_sep[9]);   //bottom[pixel]
		float h        = stringToNum<float>(temp_sep[10]);  //h[m]
		float w        = stringToNum<float>(temp_sep[11]);  //w[m]
		float l        = stringToNum<float>(temp_sep[12]);  //l[m]
		float x        = stringToNum<float>(temp_sep[13]);  //x[m]
		float y        = stringToNum<float>(temp_sep[14]);  //y[m]
		float z        = stringToNum<float>(temp_sep[15]);  //z[m]
		float yaw      = stringToNum<float>(temp_sep[16]);  //yaw angle[rad]

		if(truncation>max_truncation || occlusion>max_occlusion || classname2id[type] != classname2id[trackclass])
			continue;

		Eigen::Vector3d cpoint;
		cpoint<<x,y,z;
		//cout<<"came " <<cpoint<<"\n"<<endl;

		Eigen::Vector3d ppoint = camera2cloud(cpoint);
		//cout<<"cloud: "<< ppoint<<"\n"<<endl;
		
		Detect det;
		det.box2D.resize(4);
		det.box.resize(3);
		det.classname  = classname2id[type];
		det.box2D[0]   = x1;
		det.box2D[1]   = y1;
		det.box2D[2]   = x2;
		det.box2D[2]   = y2;
		det.box[0]     = h;//h
		det.box[1]     = w;//w
		det.box[2]     = l;//l
		det.z          = ppoint(2);
		det.yaw        = yaw;
		det.position   = Eigen::VectorXd(2);
		det.position << ppoint(0),ppoint(1);

		Inputdets[fra].push_back(det);
	}
	// start position
	Eigen::Matrix3d rotZorigion; 
	Eigen::Isometry3d porigion;
	double oriheading=0;
	double orix =0;
	double oriy = 0;

	Eigen::Matrix3d rotZpre;
	Eigen::Isometry3d ppre;
	double preheading=0;
	double prex =0;
	double prey = 0;
	cv::Mat images = cv::Mat::zeros(608,608,CV_8UC3);

	double totaltime = 0;

	int frame = 0;

	cout<<"maxfrmae "<<maxframe<<endl;


	cv::RNG rng(12345);
	unordered_map<int, vector<int>> idcolor;

	/*cv::VideoWriter writer;

	writer.open(path+"tracking.avi",CV_FOURCC('M', 'J', 'P', 'G'), 10 , cv::Size(1241, 376), true);
    	if (!writer.isOpened())
    	{
        	std::abort();
    	}*/


	while(ros::ok() && frame < maxframe){


		int max_marker_size_ = 50;
		visualization_msgs::MarkerArray marker_array;
		marker_array.markers.clear();
		visualization_msgs::Marker bbox_marker;
		bbox_marker.header.frame_id = "global_init_frame";
		bbox_marker.header.stamp = ros::Time::now();
		bbox_marker.ns = "";
		bbox_marker.lifetime = ros::Duration();
		bbox_marker.frame_locked = true;
		bbox_marker.type = visualization_msgs::Marker::CUBE;
		bbox_marker.action = visualization_msgs::Marker::ADD;



		visualization_msgs::MarkerArray text_marker_array;
		text_marker_array.markers.clear();
		visualization_msgs::Marker text_marker;
		text_marker.header.frame_id = "global_init_frame";
		text_marker.header.stamp = ros::Time::now();
		text_marker.ns = "";
		text_marker.lifetime = ros::Duration();
		text_marker.frame_locked = true;
		text_marker.color.r = 0.0f;
		text_marker.color.g = 0.0f;
		text_marker.color.b = 0.0f;
		text_marker.color.a = 0.9;
		text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		text_marker.action = visualization_msgs::Marker::ADD;


		//get the gps data and get the tranformation matrix
		tokenizer tokn(gpsdata[frame], sep);
		vector<string> temp_sep(tokn.begin(), tokn.end());

		double UTME,UTMN;
		double latitude = stringToNum<double>(temp_sep[0]);
		double longitude = stringToNum<double>(temp_sep[1]);
		double heading = stringToNum<double>(temp_sep[5]) - 90 * M_PI/180;
		LonLat2UTM(longitude, latitude, UTME, UTMN);

		Eigen::Isometry3d translate2origion;
		Eigen::Isometry3d origion2translate;

		double twosub=0;

		//read image
		string impath = path + "/image_2/"+imagename[frame];
		cv::Mat rgbimage = cv::imread(impath);

		//read cloud
		string cloudpath = path + "/velodyne/"+lidarname[frame];
		pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud(
			new pcl::PointCloud<pcl::PointXYZI>);
		LoadPointcloud(*Cloud, cloudpath);


		if(frame == 0){
			Eigen::AngleAxisd roto(heading ,Eigen::Vector3d::UnitZ());
			rotZorigion = roto.toRotationMatrix();
			porigion = rotZorigion;
			orix = UTME;
			oriy = UTMN;
			porigion.translation() = Eigen::Vector3d(UTME, UTMN, 0);
			oriheading = heading;

			preheading = heading;
			prex = UTME;
			prey = UTMN;
			rotZpre = rotZorigion;
			ppre = rotZpre;
			ppre.translation() = Eigen::Vector3d(UTME, UTMN, 0);

		}else{

			Eigen::AngleAxisd rotnow(heading , Eigen::Vector3d::UnitZ());
			Eigen::Matrix3d rotpnow = rotnow.toRotationMatrix();
			Eigen::Isometry3d p2;
			p2 = rotpnow;
			p2.translation() = Eigen::Vector3d(UTME, UTMN, 0);
			translate2origion = porigion.inverse() * p2;
			origion2translate = p2.inverse() * porigion;
			twosub = heading - oriheading;
		}

		int size = Inputdets[frame].size();
		cout<<"inpusize "<<size<<endl;

		for(int i=0; i<size; ++i){

			vector<int> color = {0,255,0};
			draw3dbox(Inputdets[frame][i], rgbimage, color);

			cout<<"input: "<<Inputdets[frame][i].position(0)<<" "<<Inputdets[frame][i].position(1)<<" "<<Inputdets[frame][i].z<<" "<<Inputdets[frame][i].box[0]<<" "<<Inputdets[frame][i].box[1]<<" "<<Inputdets[frame][i].box[2]<<endl;

			Eigen::VectorXd v(2,1);
			v(1)   = Inputdets[frame][i].position(0);//x in kitti lidar
			v(0)   = -Inputdets[frame][i].position(1);//y in kitti lidar
			if(frame!=0){
				Eigen::Vector3d p_0(v(0), v(1), 0);
				Eigen::Vector3d p_1;
				p_1 = translate2origion * p_0;
				v(0) = p_1(0);
				v(1) = p_1(1);
			}

			Inputdets[frame][i].position(0) = v(0);
			Inputdets[frame][i].position(1) = v(1);

			Inputdets[frame][i].rotbox = cv::RotatedRect(cv::Point2f((v(0)+25)*608/50, v(1)*608/50), 
							cv::Size2f(Inputdets[frame][i].box[1]*608/50, Inputdets[frame][i].box[2]*608/50), Inputdets[frame][i].yaw);				     

			cv::RotatedRect detshow = cv::RotatedRect(cv::Point2f((v(0)+25)*608/50, v(1)*608/50), 
							cv::Size2f(Inputdets[frame][i].box[1]*608/50, Inputdets[frame][i].box[2]*608/50), Inputdets[frame][i].yaw);
			cv::Point2f vertices[4];
			detshow.points(vertices);
	    		for (int j = 0; j < 4; j++)
	        		cv::line(images, vertices[j], vertices[(j+1)%4], cv::Scalar(0,0,255), 1);
		}

		std::vector<Eigen::VectorXd> result;
		int64_t tm0 = gtm();
		tracker.track(Inputdets[frame],time, result);
   		int64_t tm1 = gtm();
  		printf("[INFO]update cast time:%ld us\n",  tm1-tm0);

		double x = tm1-tm0;
		totaltime += x;

		int marker_id = 0;
		for(int i=0; i<result.size(); ++i){
			Eigen::VectorXd r = result[i];
			if(frame != 0){
				Eigen::Vector3d p_0(r(1), r(2), 0);
				Eigen::Vector3d p_1;
				p_1 = origion2translate * p_0;
				r(1) = p_1(0);
				r(2) = p_1(1);
			}

			Detect det;
			det.box2D.resize(4);
			det.box.resize(3);
			det.box[0]     = r(9);//h
			det.box[1]     = r(8);//w
			det.box[2]     = r(7);//l
			det.z          = r(10);
			det.yaw        = r(6);
			det.position   = Eigen::VectorXd(2);
			det.position << r(2),-r(1);
			cout<<"det: "<<det.position(0)<<" "<<det.position(1)<<" "<<det.z<<" "<<det.box[0]<<" "<<det.box[1]<<" "<<det.box[2]<<endl;

			if (!idcolor.count(int(r(0)))){
                		int red = rng.uniform(0, 255);
                		int green = rng.uniform(0, 255);
                		int blue = rng.uniform(0, 255);			
				idcolor[int(r(0))] = {red,green,blue};
			}
			draw3dbox(det, rgbimage, idcolor[int(r(0))]);


	                Eigen::Vector3f eulerAngle(-det.yaw, 0.0, 0.0);
	                Eigen::AngleAxisf rollAngle(Eigen::AngleAxisf(eulerAngle(2), Eigen::Vector3f::UnitX()));
	                Eigen::AngleAxisf pitchAngle(Eigen::AngleAxisf(eulerAngle(1), Eigen::Vector3f::UnitY()));
	                Eigen::AngleAxisf yawAngle(Eigen::AngleAxisf(eulerAngle(0), Eigen::Vector3f::UnitZ()));
			const Eigen::Quaternionf bboxQ1(yawAngle * pitchAngle * rollAngle);
			Eigen::Vector4f q = bboxQ1.coeffs();

			bbox_marker.id = marker_id;
			bbox_marker.pose.position.x    = det.position(0);
			bbox_marker.pose.position.y    = det.position(1);
			bbox_marker.pose.position.z    = det.z+ det.box[0]/2;
			bbox_marker.pose.orientation.x = q[0];
			bbox_marker.pose.orientation.y = q[1];
			bbox_marker.pose.orientation.z = q[2];
			bbox_marker.pose.orientation.w = q[3];
			bbox_marker.scale.x            = det.box[1];
			bbox_marker.scale.y            = det.box[2];
			bbox_marker.scale.z            = det.box[0];
			bbox_marker.color.r            = float(idcolor[int(r(0))][0])/255;
			bbox_marker.color.g            = float(idcolor[int(r(0))][1])/255;
			bbox_marker.color.b            = float(idcolor[int(r(0))][2])/255;
			bbox_marker.color.a            = 0.8;
			marker_array.markers.push_back(bbox_marker);

			text_marker.id = marker_id;
			text_marker.pose.position.x    = det.position(0);
			text_marker.pose.position.y    = det.position(1);
			text_marker.pose.position.z    = det.z + det.box[0]/2 + 1;
			text_marker.pose.orientation.x = 0;
			text_marker.pose.orientation.y = 0;
			text_marker.pose.orientation.z = 0;
			text_marker.pose.orientation.w = 1;
			text_marker.scale.x            = 0.2;
			text_marker.scale.y            = 0;
			text_marker.scale.z            = 1;
			text_marker.text = "ID: " + toString(int(r(0)));
			text_marker_array.markers.push_back(text_marker);
			++marker_id;

		}

                if (marker_array.markers.size() > max_marker_size_) {
			max_marker_size_ = marker_array.markers.size();
		}


		for (size_t i = marker_id; i < max_marker_size_; ++i) {
			bbox_marker.id = i;
			bbox_marker.color.a = 0;
			bbox_marker.pose.position.x = 0;
			bbox_marker.pose.position.y = 0;
			bbox_marker.pose.position.z = 0;
			bbox_marker.scale.x = 0;
			bbox_marker.scale.y = 0;
			bbox_marker.scale.z = 0;
			marker_array.markers.push_back(bbox_marker);

			text_marker.id = i;
			text_marker.color.a = 0;
			text_marker.pose.position.x = 0;
			text_marker.pose.position.y = 0;
			text_marker.pose.position.z = 0;
			text_marker.scale.x = 0;
			text_marker.scale.y = 0;
			text_marker.scale.z = 0;
			text_marker_array.markers.push_back(text_marker);

			++marker_id;
		}

		cv::imshow("x", rgbimage);
		cv::waitKey(1);
		//writer<<rgbimage;
		sensor_msgs::ImagePtr imgmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",rgbimage).toImageMsg(); //转换为ros接受的消息
		pubimage.publish(imgmsg);

		sensor_msgs::PointCloud2 ros_cloud;
		pcl::toROSMsg(*Cloud, ros_cloud);
		ros_cloud.header.frame_id = "global_init_frame";
		publidar.publish(ros_cloud);

		pubtextmarker.publish(text_marker_array);
		pubmarker.publish(marker_array);

		time+=0.1;
		frame++;
		r.sleep();

	}
	//writer.release();


	return 0;
}
