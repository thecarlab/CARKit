#include <rclcpp/rclcpp.hpp> 
#include <sensor_msgs/msg/laser_scan.hpp>

#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <sched.h>

#include <sys/select.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <math.h>
#include "../include/data_type.h"
#include "../include/remote.h"

#define DEG2RAD(x) ((x)*M_PI / 180.f)
// CARKit learning annotation: implements the behavior described by this file's package and module.
using namespace std;
class lakibeam1_scan : public rclcpp::Node
{
public:
	lakibeam1_scan():Node("laser_scan_publisher")
	{
		declare_parameters();
		get_parameters();
		scan_pub = create_publisher<sensor_msgs::msg::LaserScan>(output_topic, 1000);
		info();
		// scan_config();
		if (create_socket() != 0)
		{
			throw std::runtime_error("Unable to bind the LakiBeam UDP socket");
		}
		scan_begin = get_clock()->now();
		scan_publish();
	}
protected:
	void get_parameters()
	{
		get_parameter<string>("frame_id",frame_id);
		get_parameter<std::string>("port",port);
		get_parameter<string>("hostip",hostip);
		get_parameter<string>("sensorip",sensorip);
		get_parameter<string>("output_topic",output_topic);
		get_parameter<string>("scanfreq",scanfreq);
		get_parameter<string>("filter",filter);
		get_parameter<string>("laser_enable",laser_enable);
		get_parameter<string>("scan_range_start",scan_range_start);
		get_parameter<string>("scan_range_stop",scan_range_stop);
		get_parameter<bool>("inverted",inverted);
		get_parameter<int>("angle_offset",angle_offset);
	};

	void declare_parameters()
	{
		declare_parameter<string>("frame_id",frame_id);
		declare_parameter<string>("port",port);
		declare_parameter<string>("hostip",hostip);
		declare_parameter<string>("sensorip",sensorip);
		declare_parameter<string>("output_topic",output_topic);
		declare_parameter<string>("scanfreq",scanfreq);
		declare_parameter<string>("filter",filter);
		declare_parameter<string>("laser_enable",laser_enable);
		declare_parameter<string>("scan_range_start",scan_range_start);
		declare_parameter<string>("scan_range_stop",scan_range_stop);
		declare_parameter<bool>("inverted",inverted);
		declare_parameter<int>("angle_offset",angle_offset);
	};
	void info()
	{
		RCLCPP_INFO(get_logger(),"frame_id:%s", frame_id.c_str());
		RCLCPP_INFO(get_logger(),"output_topic:%s", output_topic.c_str());
		RCLCPP_INFO(get_logger(),"inverted:%s", (inverted ? "True" : "False"));
		RCLCPP_INFO(get_logger(),"hostip:%s", hostip.c_str());
		RCLCPP_INFO(get_logger(),"sensorip:%s", sensorip.c_str());
		RCLCPP_INFO(get_logger(),"port:%s", port.c_str());
		RCLCPP_INFO(get_logger(),"scanfreq:%s", scanfreq.c_str());
		RCLCPP_INFO(get_logger(),"filter:%s", filter.c_str());
		RCLCPP_INFO(get_logger(),"laser_enable:%s", laser_enable.c_str());
		RCLCPP_INFO(get_logger(),"scan_range_start:%s", scan_range_start.c_str());
		RCLCPP_INFO(get_logger(),"scan_range_stop:%s", scan_range_stop.c_str());

	};
	void scan_config()
	{
		RCLCPP_INFO(get_logger(),"scan_config");
		sensor_config(sensorip, "/api/v1/sensor/scanfreq", scanfreq);
		sensor_config(sensorip, "/api/v1/sensor/laser_enable", laser_enable);
		sensor_config(sensorip, "/api/v1/sensor/scan_range/start", scan_range_start);
		sensor_config(sensorip, "/api/v1/sensor/scan_range/stop", scan_range_stop);		
		RCLCPP_INFO(get_logger(),"scan_config1");

	};
	int create_socket()
    {
		RCLCPP_INFO(get_logger(),"create_socket");
		// rclcpp::sleep_for(std::chrono::milliseconds(2000));
		// get_telemetry_data(sensorip);
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if(sockfd == -1)
        {
            RCLCPP_INFO(get_logger(),"Failed to create socket");
            return -1;
        }

        memset(&ser_addr, 0, sizeof(ser_addr));
        ser_addr.sin_family = AF_INET;
        ser_addr.sin_addr.s_addr = inet_addr(hostip.c_str());
        ser_addr.sin_port = htons(atoi(port.c_str()));

        if(bind(sockfd, (struct sockaddr*)&ser_addr, sizeof(ser_addr)) < 0)
        {
			RCLCPP_ERROR(get_logger(),"Socket bind error!");
			close(sockfd);
            return -1;
        }
		struct timeval receive_timeout;
		receive_timeout.tv_sec = 0;
		receive_timeout.tv_usec = 200000;
		setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO,
			&receive_timeout, sizeof(receive_timeout));
        return 0;
    };
	void scan_publish()
	{
		double inf = std::numeric_limits<double>::infinity();
		RCLCPP_INFO(get_logger(),"scan_publish");
		bool synchronized = false;
		while (rclcpp::ok())
		{
			socklen_t len = sizeof(clent_addr);
			const ssize_t received = recvfrom(
				sockfd, &MSOP_Data, sizeof(MSOP_Data), 0,
				(struct sockaddr*)&clent_addr, &len);
			if(received != sizeof(MSOP_Data))
			{
				continue;
			}

			// Each block contains 16 uniformly spaced beams. Derive the
			// 0.25-degree beam resolution from adjacent valid block azimuths.
			for(int block = 0; block < 11; block++)
			{
				if(MSOP_Data.BlockID[block].DataFlag != 0xEEFF ||
					MSOP_Data.BlockID[block + 1].DataFlag != 0xEEFF)
				{
					continue;
				}
				const int difference =
					MSOP_Data.BlockID[block + 1].Azimuth -
					MSOP_Data.BlockID[block].Azimuth;
				if(difference > 0 && difference < 1000)
				{
					resolution = difference / 16;
					break;
				}
			}

			for(int block = 0; block < 12; block++)
			{
				if(MSOP_Data.BlockID[block].DataFlag != 0xEEFF)
				{
					continue;
				}
				for(int beam = 0; beam < 16; beam++)
				{
					bm_response_scan_t response;
					response.angle = MSOP_Data.BlockID[block].Azimuth +
						resolution * beam;

					if(!scan_vec.empty() && response.angle < scan_vec.back().angle)
					{
						const rclcpp::Time boundary = get_clock()->now();
						if(synchronized && !scan_vec.empty())
						{
							sensor_msgs::msg::LaserScan scan;
							const size_t count = scan_vec.size();
							const float duration = (boundary - scan_begin).seconds();
							scan.header.stamp = scan_begin;
							scan.header.frame_id = frame_id;
							scan.angle_min = DEG2RAD(-180 + angle_offset);
							scan.angle_increment = 2.0 * M_PI / count;
							scan.angle_max = scan.angle_min +
								scan.angle_increment * (count - 1);
							scan.scan_time = duration;
							scan.time_increment = duration / (float)count;
							scan.range_min = 0.0;
							scan.range_max = 100.0;
							scan.ranges.resize(count);
							scan.intensities.resize(count);

							for(size_t index = 0; index < count; index++)
							{
								const size_t target = inverted ? count - index - 1 : index;
								scan.ranges[target] = (float)scan_vec[index].dist / 1000;
								scan.intensities[target] = scan_vec[index].rssi;
								if(scan.ranges[target] == 0)
								{
									scan.ranges[target] = inf;
									scan.intensities[target] = 0;
								}
							}
							scan_pub->publish(scan);
						}
						synchronized = true;
						scan_begin = boundary;
						scan_vec.clear();
					}

					response.dist = MSOP_Data.BlockID[block].Result[beam].Dist_1;
					response.rssi = MSOP_Data.BlockID[block].Result[beam].RSSI_1;
					scan_vec.push_back(response);
				}
			}
		}
		close(sockfd);
	}

private:
    string hostip, sensorip, port, frame_id, output_topic,scanfreq,filter,laser_enable,scan_range_start,scan_range_stop;
    int resolution=25, angle_offset;
    bool inverted;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
    rclcpp::Time scan_begin;
    struct sockaddr_in ser_addr, clent_addr; 
	int sockfd;
    std::vector <bm_response_scan_t> scan_vec;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::Rate rate(30); 
	auto node = make_shared<lakibeam1_scan>();
	rclcpp::spin(node);
	rclcpp::shutdown();	

	return 0;
}
