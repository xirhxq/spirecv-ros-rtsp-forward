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

std::string suavState, usvState, tuav6State, tuav8State, tuav2State, tuav3State;

void suavStateCallback(const std_msgs::String::ConstPtr &msg) {
    if (msg->data == "COMM_TEST" || msg->data == "READY" || msg->data == "SELF_CHECK" ) return;
    suavState = msg->data;
}

void usvStateCallback(const std_msgs::String::ConstPtr &msg) {
    if (msg->data == "COMM_TEST" || msg->data == "READY" || msg->data == "SELF_CHECK" ) return;
    usvState = msg->data;
}

void tuav6StateCallback(const std_msgs::String::ConstPtr &msg) {
    if (msg->data == "COMM_TEST" || msg->data == "READY" || msg->data == "SELF_CHECK" || msg->data == "VOLOST") return;
    tuav6State = msg->data;
}

void tuav8StateCallback(const std_msgs::String::ConstPtr &msg) {
    if (msg->data == "COMM_TEST" || msg->data == "READY" || msg->data == "SELF_CHECK" || msg->data == "VOLOST" ) return;
    tuav8State = msg->data;
}

void tuav2StateCallback(const std_msgs::String::ConstPtr &msg) {
    if (msg->data == "COMM_TEST" || msg->data == "READY" || msg->data == "SELF_CHECK" ) return;
    tuav2State = msg->data;
}

void tuav3StateCallback(const std_msgs::String::ConstPtr &msg) {
    if (msg->data == "COMM_TEST" || msg->data == "READY" || msg->data == "SELF_CHECK" ) return;
    tuav3State = msg->data;
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
        std::string newState;
        if ((suavState == "COUNTDOWN" || suavState == "READY" || suavState == "COMM_TEST")) {
            newState = "";
        }
        else if (suavState == "REPORT") {
            newState = "suav";
        }
        else if ((tuav2State != "STANDBY" && tuav2State != "")) {
            newState = "tuav2";
        }
        else if ((tuav3State != "STANDBY" && tuav3State != "")) {
            newState = "tuav3";
        }
        else if ((tuav6State == "FINISH" || tuav8State == "FINISH")){
            newState = "arm";
        }
        else if ((tuav8State != "WAIT" && tuav8State != "" && tuav8State != "STANDBY" && tuav8State != "VOLOST")) {
            newState = "tuav8";
        }
        else if (tuav6State == "NOFINISH"){
            newState = "tuav8";
        }
        else if ((tuav6State != "WAIT" && tuav6State != "" && tuav6State != "STANDBY" && tuav6State != "VOLOST")) {
            newState = "tuav6";
        }
        else if ((usvState == "DOCK_STEADY" || usvState == "DOCK_ATTACH" || usvState == "DOCK_WAIT_FINAL" || usvState == "DOCK_FINAL")){
            newState = "arm";
        }
        else if ((suavState == "DOCK" || suavState == "GUIDE" || suavState == "REFIND")){
            newState = "arm";
        }
        else if ((suavState != "REPORT")) {
            newState = "";
        }
        if (newState != state) {
            state = newState;
            changeTo(state);
        } 
		cout << "NOBACKUP" << std::endl;
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

