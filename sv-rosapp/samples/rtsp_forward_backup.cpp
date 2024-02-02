#include <iostream>
#include <string>
#include <sv_world.h>
#include <thread>
#include <ros/ros.h>
#include <std_msgs/String.h>

using namespace std;
bool hasVideo;

sv::Camera cap;
std::string url = "";
std::string who = "";
std::string realWho = "";
std::string FlyEagle = "FlyEagle";
std::string defaultPicture;
std::map<std::string, std::string> urls = {};
std::map<char, std::string> keyToKey = {};
std::string newState = "";
bool close_report = false;

void camera_init() {
	cap.setWH(1280, 720);
	cap.setFps(10);
	cap.setRtspUrl(url);
	cap.open(sv::CameraType::RTSP);
	if (who.length() > 8) {
		who = who.substr(8);
	}
}

void changeTo(std::string which) {
	if (urls.find(which) != urls.end()) {
		who = FlyEagle + which;
		url = urls[which];
		std::thread tt(&camera_init);
		tt.detach();
	}
	else {
		who = "";
		url = "";
		cap.release();
	}

}

void StreamerFlagCallback(const std_msgs::String::ConstPtr &msg) {
	std::string newWho = msg->data;
	printf("%s -> %s\n", url.c_str(), newWho.c_str());
	if (realWho == newWho) return;
	realWho = newWho;
	changeTo(newWho);
}

std::string suavState, usvState, tuav6State, tuav8State, tuav2State, tuav3State, armState;

void suavStateCallback(const std_msgs::String::ConstPtr &msg) {
    suavState = msg->data;
}

void usvStateCallback(const std_msgs::String::ConstPtr &msg) {
    usvState = msg->data;
}

void tuav6StateCallback(const std_msgs::String::ConstPtr &msg) {
    tuav6State = msg->data;
}

void tuav8StateCallback(const std_msgs::String::ConstPtr &msg) {
    tuav8State = msg->data;
}

void tuav2StateCallback(const std_msgs::String::ConstPtr &msg) {
    tuav2State = msg->data;
}

void tuav3StateCallback(const std_msgs::String::ConstPtr &msg) {
    tuav3State = msg->data;
}

void armStateCallback(const std_msgs::String::ConstPtr &msg) {
    armState = msg->data;
}


int main(int argc, char *argv[]) {
	ros::init(argc, argv, "rtsp_forward");
    ros::NodeHandle nh("~");
    auto StreamerFlag = nh.subscribe("/whichToStream", 10, StreamerFlagCallback);
    auto suavStateSub = nh.subscribe("/suav/state", 10, suavStateCallback);
    auto usvStateSub = nh.subscribe("/usv/state", 10, usvStateCallback);
    auto tuav6StateSub = nh.subscribe("/tuav6/state", 10, tuav6StateCallback);
    auto tuav8StateSub = nh.subscribe("/tuav8/state", 10, tuav8StateCallback);
    auto tuav2StateSub = nh.subscribe("/tuav2/state", 10, tuav2StateCallback);
    auto tuav3StateSub = nh.subscribe("/tuav3/state", 10, tuav3StateCallback);
	auto armStateSub = nh.subscribe("/arm/state", 10, armStateCallback);
	nh.getParam("/which/state", newState);

	XmlRpc::XmlRpcValue urlsMap;
	if (nh.getParam("urls", urlsMap)) {
		for (auto it = urlsMap.begin(); it != urlsMap.end(); it++) {
			urls[it -> first] = static_cast<std::string>(it -> second);
		}
		for (const auto& kv: urls) {
			char c = 'a' + keyToKey.size();
			keyToKey[c] = kv.first.c_str();
			ROS_INFO("[%c] -> %s -> Value: %s", c, kv.first.c_str(), kv.second.c_str());
		}
	}
	else {
		ROS_ERROR("Unable to read map");
	}

	if (!nh.getParam("defaultPicture", defaultPicture)) {
		ROS_ERROR("Unable to load picture name");
	}

	//sv::VideoStreamer streamer;
	//streamer.setup(cv::Size(1280, 720), 8553, 1, "/live3");
	cv::Mat img, img_forward, kunkun;
	kunkun = cv::Mat::zeros(720, 1280, CV_8UC3);
	kunkun = cv::imread(sv::get_home() + "/spirecv-ros/sv-rosapp/pictures/" + defaultPicture);
	kunkun.copyTo(img_forward);
    
    std::string state;

	while (ros::ok()) {
		cv::imshow(FlyEagle, img_forward);
		char q = cv::waitKey(10);
		if (keyToKey.find(q) != keyToKey.end()) {
			changeTo(keyToKey[q]);
		}
		else if (q == 'z') {
			changeTo("");
		}
		if(newState != "" && newState != "suav")
		{
			close_report = true;
		}
        
        if ((suavState == "COUNTDOWN" || suavState == "READY" || suavState == "COMM_TEST")) {
            newState = "";
        }
		else if (suavState == "PREPARE" || suavState == "SEARCH") {
			newState = "";
		}
        else if (!close_report && suavState == "REPORT") {
            newState = "suav";
        }
        // if arm wait_uav6_8_takeoff and tuav6 or tuav8 wait
		// if (armState == "wait_uav6_8_takeoff")
		// {
		// 	newState = "arm";
		// }
		if (tuav6State == "TAKEOFFUSV")
		{
			newState = "tuav6";
			close_report = true;
		}
		if (tuav8State == "TAKEOFFUSV")
		{
			newState = "tuav8";
		}
		if (armState == "searching")
		{
			newState = "arm";
		}
		if (tuav2State == "TAKEOFF")
		{
			newState = "tuav2";
		}
		if (tuav3State == "TAKEOFF")
		{
			newState = "tuav3";
		}
		if (armState == "grasp_init")
		{
			newState = "arm";
		}
		nh.setParam("/which/state", newState);


        if (newState != state) {
            state = newState;
            changeTo(state);
        } 
		cout << "------BACKUP------" << std::endl;
        cout << "suav: " << suavState << std::endl; 
        cout << "usv: " << usvState << std::endl; 
        cout << "tuav2: " << tuav2State << std::endl; 
        cout << "tuav3: " << tuav3State << std::endl; 
        cout << "tuav6: " << tuav6State << std::endl; 
        cout << "tuav8: " << tuav8State << std::endl; 
		printf("State: %s, %s: %s\n", state.c_str(), who.c_str(), url.c_str());
		if (urls.find(who) != urls.end()) {
			cap.read(img);
			img.copyTo(img_forward);
		}
		else {
			kunkun.copyTo(img_forward);
		}
		cv::resize(img_forward, img_forward, cv::Size(1280, 720));
		//streamer.stream(img_forward);
		ros::spinOnce();
	}
	return 0;
}

